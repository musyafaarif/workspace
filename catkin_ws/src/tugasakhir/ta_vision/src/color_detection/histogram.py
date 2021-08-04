import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib.patches import Rectangle
import cv2 as cv
import numpy as np

FIND_HUE = 1
FIND_SAT = 2
FIND_VAL = 4
FIND_MAX = 1
FIND_MEAN = 2

ERROR_NONE = -1
ERROR_FIND = 1
ERROR_FILTER = 2

HUE_MAP = {"red": (-30, 30),
           "red_1": (0, 30),
           "red_2": (150, 180),
           "yellow": (0, 60),
           "green": (30, 90),
           "cyan": (60, 120),
           "blue": (90, 150),
           "purple": (120, 180)}


class channel:
    def __init__(self):
        self.color = "red"
        self.find_range = (0, 255)
        self.method = FIND_MAX
        self.min = 1
        self.gap = 1
        self.value = 0
        self.range = (0, 0)


class hsv:
    def __init__(self):
        self.hue = channel()
        self.sat = channel()
        self.val = channel()

    def __assign(self, img, update=True):
        self.hsv_planes = cv.split(img)
        self.shape = img.shape
        self.size_img = img.shape[0] * img.shape[1]
        if update:
            return self.update_hist()

    def update_hist(self):
        return (
            self.update_hue(),
            self.update_sat(),
            self.update_val()
        )

    def update_hue(self):
        self.h_hist = cv.calcHist(self.hsv_planes, [0], None,
                                  [180], (0, 180), accumulate=False)
        return self.h_hist

    def update_sat(self):
        self.s_hist = cv.calcHist(self.hsv_planes, [1], None,
                                  [256], (0, 256), accumulate=False)
        return self.s_hist

    def update_val(self):
        self.v_hist = cv.calcHist(self.hsv_planes, [2], None,
                                  [256], (0, 256), accumulate=False)
        return self.v_hist

    def setup_hue(self, color, method, min, gap):
        self.hue.color = color
        self.hue.find_range = HUE_MAP[color]
        self.hue.method = method
        self.hue.min = min
        self.hue.gap = gap

    def setup_sat(self, find_range, method, min, gap):
        self.sat.find_range = find_range
        self.sat.method = method
        self.sat.min = min
        self.sat.gap = gap

    def setup_val(self, find_range, method, min, gap):
        self.val.find_range = find_range
        self.val.method = method
        self.val.min = min
        self.val.gap = gap

    def find_hue(self, filter):
        return self.find(self.h_hist, self.hue, filter=filter)

    def find_sat(self):
        return self.find(self.s_hist, self.sat)

    def find_val(self):
        return self.find(self.v_hist, self.val)

    def find(self, hist, _channel, filter=None):
        find_range = _channel.find_range

        if _channel.method & FIND_MAX != 0:
            max = 0; val = -1

            for val_ in range(find_range[0], find_range[1]):
                if hist[val_] > max:
                    max = hist[val_]
                    val = val_

            minimum_value = _channel.min * max

        elif _channel.method & FIND_MEAN != 0:
            xy_sum = 0; y_sum = 0; max = 0

            for val in range(find_range[0], find_range[1]):
                xy_sum = xy_sum + hist[val] * val
                y_sum = y_sum + hist[val]

            val = int(xy_sum / y_sum) if y_sum > 0 else -1
            minimum_value = _channel.min * y_sum / \
                (find_range[1] - find_range[0])

        if val == -1:
            return (
                255,
                np.array([255, 255]),
                ERROR_FIND  # Error dalam mencari
            )

        lo = self.find_(
            hist,
            val,
            find_range[0],
            -1,
            minimum_value,
            _channel.gap
        )
        hi = self.find_(
            hist,
            val,
            find_range[1],
            1,
            minimum_value,
            _channel.gap
        )

        if filter is not None:
            area = 0
            for y in hist[lo:hi]:
                area += y

            if area < filter * self.size_img:
                return (
                    255,
                    np.array([255, 255]),
                    ERROR_FILTER
                )

        return val, np.array([lo, hi]), ERROR_NONE

    def find_(self, hist, start, end, step, min, gap):
        gap_ = gap
        val_ = start

        for val in range(start, end, step):
            if hist[val] < min:
                gap_ = gap_ - 1
            else:
                gap_ = gap
                val_ = val

            if gap_ <= 0:
                return val_

        return end

    def plot(self, img):
        fig, (image, hue_plot, sat_plot, val_plot) = plt.subplots(4)
        mng = plt.get_current_fig_manager()
        fig.canvas.set_window_title('HSV Plot')
        image.spines['top'].set_visible(False)
        image.spines['right'].set_visible(False)
        image.spines['left'].set_visible(False)
        image.spines['bottom'].set_visible(False)
        image.set_xticks([])
        image.set_yticks([])
        hue_plot.spines['top'].set_visible(False)
        hue_plot.spines['right'].set_visible(False)
        hue_plot.set_xlabel('Hue')
        hue_plot.set_ylabel('Counts')
        hue_plot.set_xticks(np.arange(0, 181, 10))
        hue_plot.set_yticks([])
        hue_plot.set_xlim(0, 180)
        hue_plot.margins(x=0.01)
        sat_plot.spines['top'].set_visible(False)
        sat_plot.spines['right'].set_visible(False)
        sat_plot.set_xlabel('Saturation')
        sat_plot.set_ylabel('Counts')
        sat_plot.set_xticks(np.arange(0, 257, 32))
        sat_plot.set_yticks([])
        sat_plot.set_xlim(0, 256)
        sat_plot.margins(x=0.01)
        val_plot.spines['top'].set_visible(False)
        val_plot.spines['right'].set_visible(False)
        val_plot.set_xlabel('Value')
        val_plot.set_ylabel('Counts')
        val_plot.set_xticks(np.arange(0, 257, 32))
        val_plot.set_yticks([])
        val_plot.set_xlim(0, 256)
        val_plot.margins(x=0.01)

        hue = np.linspace(0, 180, num=180, endpoint=False)
        sat = np.linspace(0, 256, num=256, endpoint=False)
        val = np.linspace(0, 256, num=256, endpoint=False)

        hue_max = 0
        sat_max = 0
        val_max = 0
        for y in self.h_hist[self.hue.find_range[0]: self.hue.find_range[1]]:
            if y > hue_max: hue_max = y
        for y in self.s_hist[self.sat.find_range[0]: self.sat.find_range[1]]:
            if y > sat_max: sat_max = y
        for y in self.v_hist[self.val.find_range[0]: self.val.find_range[1]]:
            if y > val_max: val_max = y

        h_hist = self.h_hist
        s_hist = self.s_hist
        v_hist = self.v_hist

        for i, y in enumerate(h_hist):
            if y > hue_max: h_hist[i] = hue_max
        for i, y in enumerate(s_hist):
            if y > sat_max: s_hist[i] = sat_max
        for i, y in enumerate(v_hist):
            if y > val_max: v_hist[i] = val_max

        image.imshow(img)

        def plot_hist(plot, x, y, interval, color1, color2):
            plot.plot(
                x[:interval[0] + 1],
                y[:interval[0] + 1],
                lw=2, color=color1
            )
            plot.plot(
                x[interval[0]:interval[1] + 1],
                y[interval[0]:interval[1] + 1],
                lw=3, color=color2
            )
            plot.plot(
                x[interval[1]:],
                y[interval[1]:],
                lw=2, color=color1
            )

        plot_hist(hue_plot, hue, h_hist, self.hue.range, 'c', 'b')
        plot_hist(sat_plot, sat, s_hist, self.sat.range, 'y', 'r')
        plot_hist(val_plot, val, v_hist, self.val.range, 'c', 'g')

        def draw_bg(plot, x, interval1, interval2, y_max):
            rects = []
            lines = []
            rects.append(Rectangle(
                (0, 0), interval1[0], y_max,
                facecolor='k', alpha=0.1
            ))
            rects.append(Rectangle(
                (interval1[1], 0), 256 - interval1[1], y_max,
                facecolor='k', alpha=0.1
            ))
            lines.append(mlines.Line2D(
                [x, x], [0, y_max], ls=':', lw=2.5, color='r'
            ))
            lines.append(mlines.Line2D(
                [interval2[0], interval2[0]], [0, y_max],
                ls=':', lw=2, color='g'
            ))
            lines.append(mlines.Line2D(
                [interval2[1], interval2[1]], [0, y_max],
                ls=':', lw=2, color='g'
            ))
            for rect in rects:
                plot.add_patch(rect)
            for line in lines:
                plot.add_line(line)

        draw_bg(
            hue_plot, self.hue.value,
            self.hue.find_range, self.hue.range, hue_max
        )
        draw_bg(
            sat_plot, self.sat.value,
            self.sat.find_range, self.sat.range, sat_max
        )
        draw_bg(
            val_plot, self.val.value,
            self.val.find_range, self.val.range, val_max
        )

        fig.suptitle('HSV Plot', fontsize=16)
        fig.tight_layout()
        plt.subplots_adjust(left=0.12, right=0.9, top=0.9, bottom=0.1)
        mng.window.state('zoomed')
        plt.show()
        plt.close(fig)
