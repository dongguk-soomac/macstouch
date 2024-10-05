import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth,  1280,  720, rs.format.z16, 30)
config.enable_stream(rs.stream.color,  1280,  720, rs.format.bgr8, 30)
config.enable_record_to_file('object_detection2.bag')


spat_filter = rs.spatial_filter(1, 1, 5, 0)          # Spatial    - edge-preserving spatial smoothing
temp_filter = rs.temporal_filter(0.2, 50.0, 7)    # Temporal   - reduces temporal noise
hole_filling = rs.hole_filling_filter(2)

# Start streaming
pipeline.start(config)

e1 = cv2.getTickCount()

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        filtered = spat_filter.process(depth_frame)
        filtered = temp_filter.process(filtered)
        filtered = hole_filling.process(filtered)

        # Convert images to numpy arrays
        depth_image = np.asanyarray(filtered.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.1), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.vstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)
        e2 = cv2.getTickCount()
        t = (e2 - e1) / cv2.getTickFrequency()
        if t>10: # change it to record what length of video you are interested in
            print("Done!")
            break

finally:

    # Stop streaming
    pipeline.stop()