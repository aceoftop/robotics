import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import sys
from loguru import logger

import numpy as np
import cv2
import serial
import time


#1. GStreamer & hailo & python bind load
try: 
    import hailo
except ImportError:
    logger.error("Hailo can't python module.")
    sys.exit(1)

Gst.init(None)

class RPiCameraHailoApp:
    def __init__(self, hef_path):
        self.hef_path = hef_path
        # .so
        self.postprocess_so = "/usr/lib/aarch64-linux-gnu/hailo/tappas/post_processes/libyolo_hailortpp_post.so"
        self.loop = GLib.MainLoop()

        # uart init
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            logger.info("uart 9600bps: ok")
        except Exception as e:
            logger.warning(f"uart fail: {e}")
            self.ser = None

        self.last_cmd_time = 0
        self.step_delay = 1.0
        
        # latest
        self.latest_bottle = None
        self.latest_gripper = None

        self.pick_state = 0 
        self.state_changed_time = 0

        logger.info("gripper Init")
        self.send_cmd('i') #gripper Init

        self.align_success_count = 0
        self.required_align_frames = 2

        self.app_start_time = time.time()

        #print("4 motor up")
        #self.send_cmd('5') # 4 motor up 

    def send_cmd(self, char_cmd):
        if self.ser and self.ser.is_open:
            self.ser.write(char_cmd.encode())
            #logger.info(f"tx : {char_cmd}")
            print(f"tx : {char_cmd}")
        else:
            #logger.warning(f"tx fail: {char_cmd}")
            print(f"tx fail: {char_cmd}")

    #def track_and_pick_with_gripper(self, bottle_box, gripper_box):
    def control_loop_callback(self):
        current_time = time.time()

        if current_time - self.app_start_time < 4.0:
            print("camera warming up")
            self.latest_bottle = None
            self.latest_gripper = None
            return True


        #state sequence control
        if self.pick_state == 1:
            if current_time - self.state_changed_time >= 1.2:
                print("2-2:motor4 up")
                self.send_cmd('5') # 4 motor up 
                self.pick_state = 2
                self.state_changed_time = current_time
            return True


        if self.pick_state == 2:
            if current_time - self.state_changed_time >= 1.2:
                print("2-3:motor2 down")
                self.send_cmd('s') # 2 motor down 
                self.pick_state = 3
                self.state_changed_time = current_time
            return True

        if self.pick_state == 3:
            if current_time - self.state_changed_time >= 1.2:
                print("2-4:close gripper")
                self.send_cmd('c') # 5 motor close
                self.pick_state = 4
                self.state_changed_time = current_time
            return True

        if self.pick_state == 4:
            if current_time - self.state_changed_time >= 1.2:
                print("3:bottle pick up")
                self.send_cmd('y')  # 2 motor up
                self.pick_state = 5
                self.align_success_count = 0
                self.last_cmd_time = current_time
            return True

        if self.pick_state == 5:
            if current_time - self.state_changed_time >= 1.5:
                print("=== sequence end ")
                self.pick_state = 5 
                self.align_success_count = 0
                self.latest_bottle = None
                self.latest_gripper = None
            return True

        # target data None so skip
        if self.latest_bottle is None or self.latest_gripper is None:
            if self.latest_bottle is None:
                print("  warning : bottole not rx buffer")
                #bottle_x = self.latest_bottle[0]
                #print(f"     b:{bottle_x}, g{self.latest_gripper[0]}")
            if self.latest_gripper is None:
                print("  warning : gripper not rx buffer")
                #print(f"     b:{self.latest_bottle[0]}, g{self.latest_gripper[0]}")
            return True

        if current_time - self.last_cmd_time < self.step_delay:
            return

        #1. x postion 
        bottle_box = self.latest_bottle
        gripper_box = self.latest_gripper


        bx, _, bw, _ = bottle_box
        gx, _, gw, _ = gripper_box

        bottle_center_x = bx + (bw // 2)
        gripper_center_x = gx + (gw // 2)

        #3: diff check 
        error_x = bottle_center_x - gripper_center_x
        #logger.info(f" bot center:{bottle_center_x}, grip center:{gripper_center_x}, off:{error_x}")
        print(f" b:{bottle_center_x}, g{gripper_center_x}, o:{error_x}")


        if error_x < -10 or error_x > 10:
            self.align_success_count = 0

            if error_x < -10:
                print("r")
                self.send_cmd('r')
            elif error_x > 10:
                print("l")
                self.send_cmd('l')
            
            self.last_cmd_time = current_time

        else:
            # count inc
            self.align_success_count += 1
            print(f" check : ({self.align_success_count}/{self.required_align_frames})")

            #capture
            if self.align_success_count >= self.required_align_frames:
                print("2-1: ready capture!")
                self.pick_state = 1 
                self.state_changed_time = current_time
                self.last_cmd_time = time.time()
        
        return True

    def app_callback(self, pad, info):
        # Gstreamer queue pad metadata buffer. get callback func
        buffer = info.get_buffer()
        if not buffer:
            print(" Not buffer")
            return Gst.PadProbeReturn.OK

        # hailo get ROI
        roi = hailo.get_roi_from_buffer(buffer)
        if not roi:
            print(" Not roi")
            return Gst.PadProbeReturn.OK

        #1.=== openCV  blue rectangle
        gripper_target = None

        sucess, map_info = buffer.map(Gst.MapFlags.READ)
        if sucess:
            try:
                width = 640
                height = 640

                frame = np.ndarray(shape=(height, width,3), dtype=np.uint8, buffer=map_info.data)

                bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)

                #Blue
                lower_blue = np.array([100, 150, 50])
                upper_blue = np.array([140, 255, 255])
                mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

                #Red
                lower_red1 = np.array([0, 120, 70])
                upper_red1 = np.array([10, 255, 255])
                lower_red2 = np.array([170, 120, 70])
                upper_red2 = np.array([180, 255, 255])

                mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
                mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
                mask_red = cv2.bitwise_or(mask_red1, mask_red2)

                color_targets = [
                        (mask_blue, "Blue"),
                        (mask_red,  "Red")
                    ]

                for mask, color_label in color_targets:

                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    for cnt in contours:
                        area = cv2.contourArea(cnt)
                        if area > 500:
                                x, y, w, h = cv2.boundingRect(cnt)

                                #if color_label == "Red" and gripper_target is None:
                                if color_label == "Red" :
                                    gripper_target = (x, y, w, h)
                                    #print(f"gripper_target : {gripper_target}")

                                x_norm = x / width
                                y_norm = y / height
                                w_norm = w / width
                                h_norm = h / height

                                hailo_bbox = hailo.HailoBBox(x_norm, y_norm, w_norm, h_norm)

                                
                                new_detection = hailo.HailoDetection(hailo_bbox, color_label, 0.99)
                                roi.add_object(new_detection)
                                #print(f"{color_label} :{x}, {y}, {w}, {y}")

            except Exception as e:
                logger.error(f"OpenCV err : {e}")
            finally:
                buffer.unmap(map_info)

        # Hailo   1D list array.
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

        bottle_count = 0
        toothbrush_count = 0
        target_bottle = None
        #print("\n===== real AI start =====")

        for det in detections:
            label = det.get_label()
            confidence = det.get_confidence()
            bbox = det.get_bbox()

            width, height = 640, 640

            x = int(bbox.xmin() * width)
            y = int(bbox.ymin() * height)
            w = int(bbox.width() * width)
            h = int(bbox.height() * height)

            # bottle
            if label.lower() == 'bottle' and confidence >= 0.30:
                bottle_count += 1
                target_bottle = (x, y, w, h)
            elif label not in ["Blue", "Red"]:
                roi.remove_object(det)

        # 3
        #if target_bottle is not None and gripper_target is not None:
        #    self.track_and_pick_with_gripper(target_bottle, gripper_target)
        #else:
        #    if target_bottle is None:
        #        print("    Not see bottle")
        #    if gripper_target is None:
        #        print("    Not see gripper")
        
        if self.pick_state == 0:
            if target_bottle is not None:
                #print("target bottle is Not None");
                self.latest_bottle = target_bottle
            if gripper_target is not None:
                #print("gripper target is Not None")
                self.latest_gripper = gripper_target

        return Gst.PadProbeReturn.OK

    def on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.EOS or t == Gst.MessageType.ERROR:
            print("quit")
            self.loop.quit()
    
    def create_pipeline(self):
        # camera => 640x640 => NPU (hailonet) -> NMS post (hailofilter)
        pipeline_str = (
                        #f"libcamerasrc ! video/x-raw, width=1536, height=864, format=NV12 ! "
                        f"libcamerasrc ! video/x-raw, width=3280, height=2464, format=NV12 ! "
                        f"queue name=queue_scale_convert ! "
                        f"videoscale ! video/x-raw, width=640, height=640 ! "
                        f"videoconvert ! video/x-raw, format=RGB ! "
                        f"hailonet hef-path={self.hef_path} output-format-type=HAILO_FORMAT_TYPE_FLOAT32 ! "
                        f"queue name=queue_filter max-size-buffers=1 leaky=downstream ! "
                        f"hailofilter so-path={self.postprocess_so} qos=false ! "
                        f"queue name=identity_queue ! "
                        f"hailooverlay ! videoconvert ! "
                        f"fpsdisplaysink video-sink=autovideosink sync=false text-overlay=true"
                        )

        logger.info("Camera v2.1 ... GStreamer ..")
        self.pipeline = Gst.parse_launch(pipeline_str)

        #bus connect err
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_bus_message)

        # buffer change
        identity_queue = self.pipeline.get_by_name("identity_queue")
        pad = identity_queue.get_static_pad("src")
        pad.add_probe(Gst.PadProbeType.BUFFER, self.app_callback)

    def run(self):
        self.create_pipeline()
        
        # 1sec
        GLib.timeout_add(1000, self.control_loop_callback)

        logger.info("pipe streaming start...")
        self.pipeline.set_state(Gst.State.PLAYING)
        try:
            self.loop.run()
            
        except KeyboardInterrupt:
            pass
        finally:
            logger.info("stream stop")
            self.pipeline.set_state(Gst.State.NULL)



if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Raspberry Pi Camera V2.1 Haiol-10H PipeLine")
    parser.add_argument("--hef", type=str, default="/usr/share/hailo-models/yolov8m_h10.hef", help="HEF file")
    args = parser.parse_args()

    if not os.path.exists(args.hef):
        logger.error(f" can't HEF model : {args.hef}")
        sys.exit(1)

    app = RPiCameraHailoApp(hef_path=args.hef)
    app.run()






