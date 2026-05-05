package org.openpnp.remote;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.imageio.ImageIO;

import org.openpnp.model.Configuration;
import org.openpnp.model.Length;
import org.openpnp.model.Location;
import org.openpnp.machine.reference.camera.OpenPnpCaptureCamera;
import org.openpnp.machine.reference.camera.OpenPnpCaptureCamera.CapturePropertyHolder;
import org.openpnp.spi.Actuator;
import org.openpnp.spi.Camera;
import org.openpnp.spi.Head;
import org.openpnp.spi.Machine;
import org.openpnp.spi.MotionPlanner.CompletionType;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.base.AbstractHead;
import org.openpnp.util.MovableUtils;
import org.pmw.tinylog.Logger;

import com.google.common.util.concurrent.FutureCallback;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonElement;
import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;

public class CodexControlServer implements AutoCloseable {
    private static final Gson GSON = new GsonBuilder().disableHtmlEscaping().create();
    private static final long COMMAND_TIMEOUT_SECONDS = 120;

    private final HttpServer server;
    private final int port;

    public static CodexControlServer createFromSystemProperties() throws IOException {
        boolean enabled = Boolean.parseBoolean(System.getProperty("codexControl.enabled", "false"));
        if (!enabled) {
            return null;
        }

        int port = Integer.parseInt(System.getProperty("codexControl.port", "5127"));
        return new CodexControlServer(port);
    }

    public CodexControlServer(int port) throws IOException {
        this.port = port;
        server = HttpServer.create(new InetSocketAddress(InetAddress.getLoopbackAddress(), port), 0);
        server.createContext("/codex/status", new StatusHandler());
        server.createContext("/codex/command", new CommandHandler());
        server.setExecutor(null);
    }

    public void start() {
        server.start();
        Logger.info("Codex control server listening on http://127.0.0.1:" + port);
    }

    @Override
    public void close() {
        server.stop(0);
    }

    private class StatusHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
                sendJson(exchange, 405, failure("Method not allowed."));
                return;
            }
            sendJson(exchange, 200, success(buildStatus()));
        }
    }

    private class CommandHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            if (!"POST".equalsIgnoreCase(exchange.getRequestMethod())) {
                sendJson(exchange, 405, failure("Method not allowed."));
                return;
            }

            try {
                CommandRequest request = readCommandRequest(exchange.getRequestBody());
                if (request == null || request.command == null || request.command.trim().isEmpty()) {
                    sendJson(exchange, 400, failure("Missing command."));
                    return;
                }
                Object result = handleCommand(request);
                sendJson(exchange, 200, success(result));
            }
            catch (IllegalArgumentException e) {
                sendJson(exchange, 400, failure(e.getMessage()));
            }
            catch (Exception e) {
                Logger.warn(e, "Codex control command failed");
                sendJson(exchange, 500, failure(e.getMessage() == null ? e.toString() : e.getMessage()));
            }
        }
    }

    private CommandRequest readCommandRequest(InputStream body) throws IOException {
        byte[] bytes = body.readAllBytes();
        if (bytes.length == 0) {
            return null;
        }
        return GSON.fromJson(new String(bytes, StandardCharsets.UTF_8), CommandRequest.class);
    }

    private Object handleCommand(CommandRequest request) throws Exception {
        String command = request.command.trim().toLowerCase(Locale.ROOT);
        switch (command) {
            case "machine.status":
                return buildStatus();
            case "config.save":
                return waitForMachineTask(() -> {
                    Configuration.get().save();
                    Map<String, Object> result = new LinkedHashMap<>();
                    result.put("saved", true);
                    result.put("configDir", Configuration.get().getConfigurationDirectory().getAbsolutePath());
                    return result;
                }, true);
            case "machine.enable":
                return waitForMachineTask(() -> {
                    boolean enabled = requireBoolean(request.enabled, "Missing enabled=true/false.");
                    Machine machine = Configuration.get().getMachine();
                    machine.setEnabled(enabled);
                    return buildStatus();
                }, true);
            case "machine.home":
                return waitForMachineTask(() -> {
                    Machine machine = Configuration.get().getMachine();
                    machine.home();
                    return buildStatus();
                }, false);
            case "head.safez":
                return waitForMachineTask(() -> {
                    Head head = findHead(request.head);
                    head.moveToSafeZ();
                    head.getMachine().getMotionPlanner().waitForCompletion(null, CompletionType.WaitForStillstand);
                    return headInfo(head);
                }, false);
            case "nozzle.safez":
                return waitForMachineTask(() -> {
                    Nozzle nozzle = findNozzle(request.nozzle, request.head);
                    nozzle.moveToSafeZ();
                    nozzle.waitForCompletion(CompletionType.WaitForStillstand);
                    MovableUtils.fireTargetedUserAction(nozzle);
                    return nozzleInfo(nozzle);
                }, false);
            case "nozzle.move":
                return waitForMachineTask(() -> {
                    Nozzle nozzle = findNozzle(request.nozzle, request.head);
                    Location target = buildAbsoluteLocation(nozzle, request);
                    if (Boolean.TRUE.equals(request.safeMove)) {
                        MovableUtils.moveToLocationAtSafeZ(nozzle, target);
                    }
                    else {
                        nozzle.moveTo(target);
                    }
                    nozzle.waitForCompletion(CompletionType.WaitForStillstand);
                    MovableUtils.fireTargetedUserAction(nozzle);
                    return nozzleInfo(nozzle);
                }, false);
            case "actuator.set":
                return waitForMachineTask(() -> {
                    Actuator actuator = findActuator(request.name);
                    actuate(actuator, request);
                    return actuatorInfo(actuator);
                }, false);
            case "actuator.read":
                return waitForMachineTask(() -> {
                    Actuator actuator = findActuator(request.name);
                    String value;
                    if (request.value != null && !request.value.isJsonNull()) {
                        value = actuator.read(actuatorValue(request.value));
                    }
                    else {
                        value = actuator.read();
                    }
                    Map<String, Object> result = actuatorInfo(actuator);
                    result.put("value", value);
                    result.put("numericValue", numericValue(value));
                    return result;
                }, false);
            case "camera.move":
                return waitForMachineTask(() -> {
                    Camera camera = findCamera(request.camera);
                    if (camera.getHead() == null) {
                        throw new IllegalArgumentException("Camera " + camera.getName() + " is fixed and cannot be moved.");
                    }
                    Location target = buildAbsoluteLocation(camera, request);
                    MovableUtils.moveToLocationAtSafeZ(camera, target);
                    camera.waitForCompletion(CompletionType.WaitForStillstand);
                    MovableUtils.fireTargetedUserAction(camera);
                    return cameraInfo(camera);
                }, false);
            case "camera.move_to_fiducial":
                return waitForMachineTask(() -> {
                    Camera camera = findMovableCamera(request.camera);
                    Location target = getFiducialLocation(camera.getHead(), request.fiducial);
                    MovableUtils.moveToLocationAtSafeZ(camera, target);
                    camera.waitForCompletion(CompletionType.WaitForStillstand);
                    MovableUtils.fireTargetedUserAction(camera);
                    return cameraInfo(camera);
                }, false);
            case "camera.capture":
                return waitForMachineTask(() -> {
                    Camera camera = findCamera(request.camera);
                    BufferedImage image = Boolean.TRUE.equals(request.light)
                            ? camera.lightSettleAndCapture()
                            : camera.settleAndCapture();
                    File outputFile = resolveCapturePath(request.path, camera.getName());
                    outputFile.getParentFile().mkdirs();
                    ImageIO.write(image, "png", outputFile);
                    Map<String, Object> result = new LinkedHashMap<>();
                    result.put("camera", camera.getName());
                    result.put("path", outputFile.getAbsolutePath());
                    result.put("width", image.getWidth());
                    result.put("height", image.getHeight());
                    return result;
                }, false);
            case "camera.properties":
                return waitForMachineTask(() -> cameraPropertiesInfo(findCaptureCamera(request.camera)), true);
            case "camera.property.set":
                return waitForMachineTask(() -> {
                    OpenPnpCaptureCamera camera = findCaptureCamera(request.camera);
                    CapturePropertyHolder holder = findCameraProperty(camera, request.property);
                    if (request.auto == null && request.propertyValue == null) {
                        throw new IllegalArgumentException("Missing property value and/or auto flag.");
                    }
                    if (request.auto != null) {
                        if (!holder.isAutoSupported()) {
                            throw new IllegalArgumentException("Property " + request.property + " does not support auto mode.");
                        }
                        holder.setAuto(request.auto.booleanValue());
                    }
                    if (request.propertyValue != null) {
                        holder.setValue(request.propertyValue.intValue());
                    }
                    if (Boolean.TRUE.equals(request.save)) {
                        Configuration.get().save();
                    }
                    return singleCameraPropertyInfo(request.property, holder);
                }, false);
            default:
                throw new IllegalArgumentException("Unsupported command: " + request.command);
        }
    }

    private <T> T waitForMachineTask(java.util.concurrent.Callable<T> callable, boolean ignoreEnabled)
            throws Exception {
        Future<T> future = Configuration.get().getMachine().submit(callable, (FutureCallback<T>) null,
                ignoreEnabled);
        try {
            return future.get(COMMAND_TIMEOUT_SECONDS, TimeUnit.SECONDS);
        }
        catch (ExecutionException e) {
            if (e.getCause() instanceof Exception) {
                throw (Exception) e.getCause();
            }
            throw new Exception(e.getCause());
        }
        catch (TimeoutException e) {
            future.cancel(true);
            throw new Exception("Command timed out after " + COMMAND_TIMEOUT_SECONDS + " seconds.", e);
        }
    }

    private Camera findMovableCamera(String cameraName) throws Exception {
        Camera camera = findCamera(cameraName);
        if (camera.getHead() == null) {
            throw new IllegalArgumentException("Camera " + camera.getName() + " is fixed and cannot be moved.");
        }
        return camera;
    }

    private Camera findCamera(String cameraName) throws Exception {
        Machine machine = Configuration.get().getMachine();
        if (cameraName == null || cameraName.trim().isEmpty()) {
            return machine.getDefaultHead().getDefaultCamera();
        }

        for (Camera camera : machine.getAllCameras()) {
            if (camera.getName().equalsIgnoreCase(cameraName.trim())) {
                return camera;
            }
        }
        throw new IllegalArgumentException("Unknown camera: " + cameraName);
    }

    private Nozzle findNozzle(String nozzleName, String headName) throws Exception {
        Machine machine = Configuration.get().getMachine();
        String trimmedNozzleName = nozzleName == null ? "" : nozzleName.trim();
        if (trimmedNozzleName.isEmpty()) {
            return findHead(headName).getDefaultNozzle();
        }

        if (headName != null && !headName.trim().isEmpty()) {
            Head head = findHead(headName);
            Nozzle nozzle = head.getNozzleByName(trimmedNozzleName);
            if (nozzle != null) {
                return nozzle;
            }
            throw new IllegalArgumentException("Unknown nozzle on head " + head.getName() + ": " + nozzleName);
        }

        for (Head head : machine.getHeads()) {
            Nozzle nozzle = head.getNozzleByName(trimmedNozzleName);
            if (nozzle != null) {
                return nozzle;
            }
        }
        throw new IllegalArgumentException("Unknown nozzle: " + nozzleName);
    }

    private OpenPnpCaptureCamera findCaptureCamera(String cameraName) throws Exception {
        Camera camera = findCamera(cameraName);
        if (!(camera instanceof OpenPnpCaptureCamera)) {
            throw new IllegalArgumentException(
                    "Camera " + camera.getName() + " does not support OpenPnP capture properties.");
        }
        return (OpenPnpCaptureCamera) camera;
    }

    private Actuator findActuator(String actuatorName) {
        if (actuatorName == null || actuatorName.trim().isEmpty()) {
            throw new IllegalArgumentException("Missing actuator name.");
        }

        Machine machine = Configuration.get().getMachine();
        Actuator actuator = machine.getActuatorByName(actuatorName);
        if (actuator != null) {
            return actuator;
        }

        for (Head head : machine.getHeads()) {
            actuator = head.getActuatorByName(actuatorName);
            if (actuator != null) {
                return actuator;
            }
        }

        throw new IllegalArgumentException("Unknown actuator: " + actuatorName);
    }

    private Head findHead(String headName) throws Exception {
        Machine machine = Configuration.get().getMachine();
        if (headName == null || headName.trim().isEmpty()) {
            return machine.getDefaultHead();
        }

        Head head = machine.getHeadByName(headName.trim());
        if (head == null) {
            throw new IllegalArgumentException("Unknown head: " + headName);
        }
        return head;
    }

    private void actuate(Actuator actuator, CommandRequest request) throws Exception {
        if (request.value == null) {
            if (request.enabled != null) {
                actuator.actuate(request.enabled.booleanValue());
                return;
            }
            throw new IllegalArgumentException("Missing actuator value.");
        }

        if (request.value.isJsonPrimitive()) {
            if (request.value.getAsJsonPrimitive().isBoolean()) {
                actuator.actuate(request.value.getAsBoolean());
                return;
            }
            if (request.value.getAsJsonPrimitive().isNumber()) {
                actuator.actuate(request.value.getAsDouble());
                return;
            }
            if (request.value.getAsJsonPrimitive().isString()) {
                actuator.actuate(request.value.getAsString());
                return;
            }
        }

        throw new IllegalArgumentException("Unsupported actuator value type.");
    }

    private Object actuatorValue(JsonElement value) {
        if (value == null || value.isJsonNull()) {
            return null;
        }

        if (value.isJsonPrimitive()) {
            if (value.getAsJsonPrimitive().isBoolean()) {
                return value.getAsBoolean();
            }
            if (value.getAsJsonPrimitive().isNumber()) {
                return value.getAsDouble();
            }
            if (value.getAsJsonPrimitive().isString()) {
                return value.getAsString();
            }
        }

        throw new IllegalArgumentException("Unsupported actuator value type.");
    }

    private Double numericValue(String value) {
        if (value == null) {
            return null;
        }
        try {
            return Double.valueOf(value.trim());
        }
        catch (NumberFormatException e) {
            return null;
        }
    }

    private Location buildAbsoluteLocation(Camera camera, CommandRequest request) {
        Location current = camera.getLocation();
        double x = request.x != null ? request.x.doubleValue() : current.getX();
        double y = request.y != null ? request.y.doubleValue() : current.getY();
        double z = request.z != null ? request.z.doubleValue() : current.getZ();
        double rotation = request.rotation != null ? request.rotation.doubleValue()
                : current.getRotation();
        return new Location(current.getUnits(), x, y, z, rotation);
    }

    private Location buildAbsoluteLocation(Nozzle nozzle, CommandRequest request) {
        Location current = nozzle.getLocation();
        double x = request.x != null ? request.x.doubleValue() : current.getX();
        double y = request.y != null ? request.y.doubleValue() : current.getY();
        double z = request.z != null ? request.z.doubleValue() : current.getZ();
        double rotation = request.rotation != null ? request.rotation.doubleValue()
                : current.getRotation();
        return new Location(current.getUnits(), x, y, z, rotation);
    }

    private Location getFiducialLocation(Head head, String fiducialName) {
        if (!(head instanceof AbstractHead)) {
            throw new IllegalArgumentException("Head " + head.getName() + " does not expose fiducial locations.");
        }

        AbstractHead abstractHead = (AbstractHead) head;
        String normalized = (fiducialName == null ? "homing" : fiducialName.trim().toLowerCase(Locale.ROOT));
        Location location;
        switch (normalized) {
            case "homing":
                location = abstractHead.getHomingFiducialLocation();
                break;
            case "primary":
                location = abstractHead.getCalibrationPrimaryFiducialLocation();
                break;
            case "secondary":
                location = abstractHead.getCalibrationSecondaryFiducialLocation();
                break;
            default:
                throw new IllegalArgumentException(
                        "Unsupported fiducial target: " + fiducialName + ". Use homing, primary, or secondary.");
        }

        if (location == null || !location.isInitialized()) {
            throw new IllegalArgumentException("Fiducial location " + normalized + " is not initialized.");
        }
        return location;
    }

    private File resolveCapturePath(String requestedPath, String cameraName) {
        if (requestedPath != null && !requestedPath.trim().isEmpty()) {
            return new File(requestedPath);
        }

        File captureDir = new File(Configuration.get().getConfigurationDirectory(), "codex-captures");
        String safeCameraName = cameraName.replaceAll("[^A-Za-z0-9._-]", "_");
        return new File(captureDir, safeCameraName + "-" + System.currentTimeMillis() + ".png");
    }

    private Map<String, Object> cameraPropertiesInfo(OpenPnpCaptureCamera camera) {
        Map<String, Object> info = cameraInfo(camera);
        info.put("freezeProperties", camera.isFreezeProperties());

        Map<String, Object> properties = new LinkedHashMap<>();
        properties.put("back-light-compensation", singleCameraPropertyInfo("back-light-compensation",
                camera.getBackLightCompensation()));
        properties.put("brightness", singleCameraPropertyInfo("brightness", camera.getBrightness()));
        properties.put("contrast", singleCameraPropertyInfo("contrast", camera.getContrast()));
        properties.put("exposure", singleCameraPropertyInfo("exposure", camera.getExposure()));
        properties.put("focus", singleCameraPropertyInfo("focus", camera.getFocus()));
        properties.put("gain", singleCameraPropertyInfo("gain", camera.getGain()));
        properties.put("gamma", singleCameraPropertyInfo("gamma", camera.getGamma()));
        properties.put("hue", singleCameraPropertyInfo("hue", camera.getHue()));
        properties.put("power-line-frequency",
                singleCameraPropertyInfo("power-line-frequency", camera.getPowerLineFrequency()));
        properties.put("saturation", singleCameraPropertyInfo("saturation", camera.getSaturation()));
        properties.put("sharpness", singleCameraPropertyInfo("sharpness", camera.getSharpness()));
        properties.put("white-balance", singleCameraPropertyInfo("white-balance", camera.getWhiteBalance()));
        properties.put("zoom", singleCameraPropertyInfo("zoom", camera.getZoom()));
        info.put("properties", properties);
        return info;
    }

    private CapturePropertyHolder findCameraProperty(OpenPnpCaptureCamera camera, String propertyName) {
        String normalized = normalizePropertyName(propertyName);
        switch (normalized) {
            case "back-light-compensation":
                return camera.getBackLightCompensation();
            case "brightness":
                return camera.getBrightness();
            case "contrast":
                return camera.getContrast();
            case "exposure":
                return camera.getExposure();
            case "focus":
                return camera.getFocus();
            case "gain":
                return camera.getGain();
            case "gamma":
                return camera.getGamma();
            case "hue":
                return camera.getHue();
            case "power-line-frequency":
                return camera.getPowerLineFrequency();
            case "saturation":
                return camera.getSaturation();
            case "sharpness":
                return camera.getSharpness();
            case "white-balance":
                return camera.getWhiteBalance();
            case "zoom":
                return camera.getZoom();
            default:
                throw new IllegalArgumentException("Unsupported camera property: " + propertyName);
        }
    }

    private String normalizePropertyName(String propertyName) {
        if (propertyName == null || propertyName.trim().isEmpty()) {
            throw new IllegalArgumentException("Missing camera property name.");
        }
        return propertyName.trim().toLowerCase(Locale.ROOT).replace('_', '-').replace(' ', '-');
    }

    private Map<String, Object> singleCameraPropertyInfo(String propertyName, CapturePropertyHolder holder) {
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("name", normalizePropertyName(propertyName));
        info.put("supported", holder.isSupported());
        info.put("autoSupported", holder.isAutoSupported());
        if (holder.isSupported()) {
            info.put("min", holder.getMin());
            info.put("max", holder.getMax());
            info.put("default", holder.getDefault());
            info.put("value", holder.getValue());
            if (holder.isAutoSupported()) {
                info.put("auto", holder.isAuto());
            }
        }
        return info;
    }

    private Map<String, Object> buildStatus() {
        Machine machine = Configuration.get().getMachine();
        Map<String, Object> status = new LinkedHashMap<>();
        status.put("enabled", machine.isEnabled());
        status.put("homed", machine.isHomed());
        status.put("busy", machine.isBusy());
        status.put("configDir", Configuration.get().getConfigurationDirectory().getAbsolutePath());

        List<Map<String, Object>> heads = new ArrayList<>();
        for (Head head : machine.getHeads()) {
            heads.add(headInfo(head));
        }
        status.put("heads", heads);

        List<Map<String, Object>> cameras = new ArrayList<>();
        for (Camera camera : machine.getAllCameras()) {
            cameras.add(cameraInfo(camera));
        }
        status.put("cameras", cameras);

        List<Map<String, Object>> actuators = new ArrayList<>();
        for (Actuator actuator : machine.getAllActuators()) {
            actuators.add(actuatorInfo(actuator));
        }
        status.put("actuators", actuators);

        return status;
    }

    private Map<String, Object> headInfo(Head head) {
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("name", head.getName());
        try {
            info.put("defaultCamera", head.getDefaultCamera().getName());
        }
        catch (Exception e) {
            info.put("defaultCamera", null);
        }
        try {
            info.put("defaultNozzle", head.getDefaultNozzle().getName());
        }
        catch (Exception e) {
            info.put("defaultNozzle", null);
        }
        List<Map<String, Object>> nozzles = new ArrayList<>();
        for (Nozzle nozzle : head.getNozzles()) {
            nozzles.add(nozzleInfo(nozzle));
        }
        info.put("nozzles", nozzles);
        if (head instanceof AbstractHead) {
            AbstractHead abstractHead = (AbstractHead) head;
            info.put("homingFiducial", locationInfo(abstractHead.getHomingFiducialLocation()));
            info.put("primaryFiducial", locationInfo(abstractHead.getCalibrationPrimaryFiducialLocation()));
            info.put("secondaryFiducial", locationInfo(abstractHead.getCalibrationSecondaryFiducialLocation()));
        }
        return info;
    }

    private Map<String, Object> cameraInfo(Camera camera) {
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("name", camera.getName());
        info.put("looking", camera.getLooking().name());
        info.put("head", camera.getHead() == null ? null : camera.getHead().getName());
        info.put("location", locationInfo(camera.getLocation()));
        return info;
    }

    private Map<String, Object> nozzleInfo(Nozzle nozzle) {
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("name", nozzle.getName());
        info.put("head", nozzle.getHead() == null ? null : nozzle.getHead().getName());
        info.put("location", locationInfo(nozzle.getLocation()));
        info.put("nozzleTip", nozzle.getNozzleTip() == null ? null : nozzle.getNozzleTip().getName());
        info.put("part", nozzle.getPart() == null ? null : nozzle.getPart().getName());
        return info;
    }

    private Map<String, Object> actuatorInfo(Actuator actuator) {
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("name", actuator.getName());
        info.put("head", actuator.getHead() == null ? null : actuator.getHead().getName());
        info.put("valueType", actuator.getValueType().name());
        return info;
    }

    private Map<String, Object> locationInfo(Location location) {
        if (location == null) {
            return null;
        }

        Map<String, Object> info = new LinkedHashMap<>();
        info.put("units", location.getUnits().name());
        info.put("x", location.getX());
        info.put("y", location.getY());
        info.put("z", location.getZ());
        info.put("rotation", location.getRotation());
        return info;
    }

    private boolean requireBoolean(Boolean value, String message) {
        if (value == null) {
            throw new IllegalArgumentException(message);
        }
        return value.booleanValue();
    }

    private Map<String, Object> success(Object result) {
        Map<String, Object> response = new LinkedHashMap<>();
        response.put("ok", true);
        response.put("result", result);
        return response;
    }

    private Map<String, Object> failure(String message) {
        Map<String, Object> response = new LinkedHashMap<>();
        response.put("ok", false);
        response.put("error", message);
        return response;
    }

    private void sendJson(HttpExchange exchange, int statusCode, Object body) throws IOException {
        byte[] bytes = GSON.toJson(body).getBytes(StandardCharsets.UTF_8);
        Headers headers = exchange.getResponseHeaders();
        headers.set("Content-Type", "application/json; charset=utf-8");
        exchange.sendResponseHeaders(statusCode, bytes.length);
        try (OutputStream output = exchange.getResponseBody()) {
            output.write(bytes);
        }
    }

    private static class CommandRequest {
        String command;
        String camera;
        String head;
        String nozzle;
        String fiducial;
        String property;
        String name;
        String path;
        Double x;
        Double y;
        Double z;
        Double rotation;
        Boolean enabled;
        Boolean light;
        Boolean auto;
        Boolean save;
        Boolean safeMove;
        Integer propertyValue;
        JsonElement value;
    }
}
