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
 * The 2026 field AprilTag layouts, bundled with the feature.
 *
 * <h2>Why {@code AprilTagFields} is not enough</h2>
 * The 2026 Rebuilt field landed in WPILib <b>2026.2.2</b>. A project on 2026.1.1 --- which is what
 * GradleRIO ships by default right now --- only goes up to {@code k2025ReefscapeWelded}, so
 * {@code AprilTagFields.kDefaultField} silently returns the <b>2025</b> field. The tags end up in
 * the wrong place and everything else looks like it works.
 *
 * <p>So the feature carries its own copy of the official WPILib JSON files and uses them by
 * default. Once you move to 2026.2.2 or later, {@code AprilTagFields.k2026RebuiltWelded} does the
 * same thing and this becomes redundant; until then, you get the right field without touching the
 * toolchain.
 *
 * <p>The files are the ones from {@code wpilibsuite/allwpilib} at tag {@code v2026.2.2},
 * unmodified: 32 tags, a 16.541 x 8.069 m field in the welded variant.
 */
public final class FieldLayouts {

    private static final String RESOURCE_DIR = "/com/stzteam/features/robotlocalization/fields/";

    private static AprilTagFieldLayout weldedCache;
    private static AprilTagFieldLayout andyMarkCache;

    private FieldLayouts() {}

    /**
     * The 2026 Rebuilt field, welded variant. This is the one used at most events.
     *
     * @return the layout, cached after the first load.
     */
    public static synchronized AprilTagFieldLayout rebuilt2026Welded() {
        if (weldedCache == null) {
            weldedCache = load("2026-rebuilt-welded.json");
        }
        return weldedCache;
    }

    /**
     * The 2026 Rebuilt field, AndyMark variant. It differs by a few centimetres in the dimensions
     * and in the position of some tags.
     *
     * @return the layout, cached after the first load.
     */
    public static synchronized AprilTagFieldLayout rebuilt2026AndyMark() {
        if (andyMarkCache == null) {
            andyMarkCache = load("2026-rebuilt-andymark.json");
        }
        return andyMarkCache;
    }

    /**
     * The field the feature uses when nobody says otherwise.
     *
     * @return the 2026 welded field.
     */
    public static AprilTagFieldLayout defaultField() {
        return rebuilt2026Welded();
    }

    /**
     * Loads a layout from your own file, in case your event uses a surveyed one.
     *
     * @param path path to the JSON in {@code AprilTagFieldLayout} format.
     * @return the layout.
     * @throws UncheckedIOException if the file cannot be read or is not in the expected format.
     */
    public static AprilTagFieldLayout fromFile(Path path) {
        try {
            return new AprilTagFieldLayout(path);
        } catch (IOException e) {
            throw new UncheckedIOException("Could not load the layout from " + path, e);
        }
    }

    private static AprilTagFieldLayout load(String fileName) {
        // AprilTagFieldLayout.loadFromResource is not used: it looks in WPILib's classloader, not
        // in this feature's, and would not find a resource bundled in our jar.
        try (InputStream stream = FieldLayouts.class.getResourceAsStream(RESOURCE_DIR + fileName)) {
            if (stream == null) {
                throw new IOException("Resource " + RESOURCE_DIR + fileName + " is not in the jar");
            }

            Path temp = Files.createTempFile("mars-field-", ".json");
            temp.toFile().deleteOnExit();
            Files.writeString(
                    temp,
                    new String(stream.readAllBytes(), StandardCharsets.UTF_8),
                    StandardCharsets.UTF_8);

            return new AprilTagFieldLayout(temp);
        } catch (IOException e) {
            throw new UncheckedIOException("Could not load field " + fileName, e);
        }
    }
}
