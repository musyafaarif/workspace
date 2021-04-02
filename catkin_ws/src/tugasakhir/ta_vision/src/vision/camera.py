#!/usr/bin/python

import cv2 as cv
import rospy

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

import imutils
from imutils.video import VideoStream

class RealCam(object):
    def __init__(self, src, fps=30, res=(320, 240)):
        self.res = res
        self.cap = VideoStream(src=src, resolution=res, framerate=fps)
        self.cap.stream.stream.set(cv.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.stream.stream.set(cv.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.stream.stream.set(cv.CAP_PROP_FPS, fps)
        self.cap.start()

    def capture(self):
        return self.cap.read()

    def close(self):
        self.cap.stop()

class SimCam():
    def __init__(self, port=5600):
        Gst.init(None)

        self.port = port
        self._frame = None

        self.video_source = 'udpsrc port={}'.format(self.port)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        self.video_decode = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        self.video_sink_conf = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config):
        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def capture(self):
        return self._frame

    def run(self):
        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK

class Camera(object):
    def __init__(self, src=None, port=None):
        if src is None and port is None:
            print("Camera object not created!")
            Exception(self)

        if not src is None:
            self.cam = RealCam(src=src)
        
        if not port is None:
            self.cam = SimCam(port)

    def capture(self):
        return self.cam.capture()

    def close(self):
        if isinstance(self.cam, RealCam):
            self.cam.close()

if __name__ == "__main__":
    cap = Camera(src=0)
    while True:
        frame = cap.capture()

        if not frame is None:
            cv.imshow("DEBUG", frame)

        k = cv.waitKey(30) & 0xff
        if k == 27:
            break

    cap.close()
    cv.destroyAllWindows()
