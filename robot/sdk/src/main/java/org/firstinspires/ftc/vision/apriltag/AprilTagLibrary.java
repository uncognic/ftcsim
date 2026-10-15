package org.firstinspires.ftc.vision.apriltag;

import java.util.LinkedHashMap;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** A set of known tags. Detections of tags in the library get metadata and a pose. */
public class AprilTagLibrary {
    private final Map<Integer, AprilTagMetadata> tags;

    private AprilTagLibrary(Map<Integer, AprilTagMetadata> tags) {
        this.tags = tags;
    }

    public AprilTagMetadata lookupTag(int id) {
        return tags.get(id);
    }

    public AprilTagMetadata[] getAllTags() {
        return tags.values().toArray(new AprilTagMetadata[0]);
    }

    public static class Builder {
        private final Map<Integer, AprilTagMetadata> tags = new LinkedHashMap<>();
        private boolean allowOverwrite;

        public Builder setAllowOverwrite(boolean allowOverwrite) {
            this.allowOverwrite = allowOverwrite;
            return this;
        }

        public Builder addTag(AprilTagMetadata tag) {
            if (!allowOverwrite && tags.containsKey(tag.id)) {
                throw new RuntimeException("You attempted to add a tag to the library when it already contains a tag with that ID. You can call .setAllowOverwrite(true) to allow overwriting the existing entry");
            }
            tags.put(tag.id, tag);
            return this;
        }

        public Builder addTag(int id, String name, double size, DistanceUnit distanceUnit) {
            return addTag(new AprilTagMetadata(id, name, size, distanceUnit));
        }

        public Builder addTags(AprilTagLibrary library) {
            for (AprilTagMetadata tag : library.getAllTags()) addTag(tag);
            return this;
        }

        public AprilTagLibrary build() {
            return new AprilTagLibrary(new LinkedHashMap<>(tags));
        }
    }
}
