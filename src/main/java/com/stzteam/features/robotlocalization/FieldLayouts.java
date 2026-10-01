// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Los layouts de AprilTags del campo 2026, empaquetados con la feature.
 *
 * <h2>Por que no basta con {@code AprilTagFields}</h2>
 * El campo 2026 Rebuilt aparecio en WPILib <b>2026.2.2</b>. Un proyecto en 2026.1.1 --- que es lo
 * que trae GradleRIO por defecto ahora mismo --- solo llega hasta
 * {@code k2025ReefscapeWelded}, asi que {@code AprilTagFields.kDefaultField} devuelve el campo de
 * <b>2025</b> sin avisar. Los tags salen donde no van y todo lo demas parece funcionar.
 *
 * <p>Asi que la feature trae su propia copia de los JSON oficiales de WPILib, y los usa por defecto.
 * Cuando subas a 2026.2.2 o mas, {@code AprilTagFields.k2026RebuiltWelded} hace lo mismo y esto
 * sobra; mientras tanto, no hay que tocar el toolchain para tener el campo correcto.
 *
 * <p>Los archivos son los de {@code wpilibsuite/allwpilib} en la etiqueta {@code v2026.2.2}, sin
 * modificar: 32 tags, campo de 16.541 x 8.069 m en la variante soldada.
 */
public final class FieldLayouts {

    private static final String RESOURCE_DIR = "/com/stzteam/features/robotlocalization/fields/";

    private static AprilTagFieldLayout weldedCache;
    private static AprilTagFieldLayout andyMarkCache;

    private FieldLayouts() {}

    /**
     * Campo 2026 Rebuilt, variante soldada. Es la que se usa en la mayoria de los eventos.
     *
     * @return el layout, cacheado tras la primera carga.
     */
    public static synchronized AprilTagFieldLayout rebuilt2026Welded() {
        if (weldedCache == null) {
            weldedCache = load("2026-rebuilt-welded.json");
        }
        return weldedCache;
    }

    /**
     * Campo 2026 Rebuilt, variante AndyMark. Difiere unos centimetros en las dimensiones y en la
     * posicion de algunos tags.
     *
     * @return el layout, cacheado tras la primera carga.
     */
    public static synchronized AprilTagFieldLayout rebuilt2026AndyMark() {
        if (andyMarkCache == null) {
            andyMarkCache = load("2026-rebuilt-andymark.json");
        }
        return andyMarkCache;
    }

    /**
     * El campo que la feature usa cuando nadie dice otra cosa.
     *
     * @return el campo 2026 soldado.
     */
    public static AprilTagFieldLayout defaultField() {
        return rebuilt2026Welded();
    }

    /**
     * Carga un layout desde un archivo propio, por si tu evento usa uno medido.
     *
     * @param path ruta al JSON en formato {@code AprilTagFieldLayout}.
     * @return el layout.
     * @throws UncheckedIOException si el archivo no se puede leer o no tiene el formato esperado.
     */
    public static AprilTagFieldLayout fromFile(Path path) {
        try {
            return new AprilTagFieldLayout(path);
        } catch (IOException e) {
            throw new UncheckedIOException("No se pudo cargar el layout de " + path, e);
        }
    }

    private static AprilTagFieldLayout load(String fileName) {
        // No se usa AprilTagFieldLayout.loadFromResource: ese busca en el classloader de WPILib, no
        // en el de esta feature, y no encontraria un recurso empaquetado en nuestro jar.
        try (InputStream stream = FieldLayouts.class.getResourceAsStream(RESOURCE_DIR + fileName)) {
            if (stream == null) {
                throw new IOException("El recurso " + RESOURCE_DIR + fileName + " no esta en el jar");
            }

            Path temp = Files.createTempFile("mars-field-", ".json");
            temp.toFile().deleteOnExit();
            Files.writeString(
                    temp,
                    new String(stream.readAllBytes(), StandardCharsets.UTF_8),
                    StandardCharsets.UTF_8);

            return new AprilTagFieldLayout(temp);
        } catch (IOException e) {
            throw new UncheckedIOException("No se pudo cargar el campo " + fileName, e);
        }
    }
}
