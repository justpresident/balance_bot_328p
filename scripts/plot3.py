#!/usr/bin/python

import matplotlib.pyplot as plt
import numpy as np
import sys
import time


class BlitManager:
    """
    BlitManager taken from
    https://matplotlib.org/stable/users/explain/animations/blitting.html
    """

    def __init__(self, canvas, animated_artists=()):
        """
        Parameters
        ----------
        canvas : FigureCanvasAgg
            The canvas to work with, this only works for subclasses of the Agg
            canvas which have the `~FigureCanvasAgg.copy_from_bbox` and
            `~FigureCanvasAgg.restore_region` methods.

        animated_artists : Iterable[Artist]
            List of the artists to manage
        """
        self.canvas = canvas
        self._bg = None
        self._artists = []

        for a in animated_artists:
            self.add_artist(a)
        # grab the background on every draw
        self.cid = canvas.mpl_connect("draw_event", self.on_draw)
        self.cid = canvas.mpl_connect("close_event", self.on_close)
        self.closed = False

    def on_close(self, event):
        self.closed = True

    def on_draw(self, event):
        """Callback to register with 'draw_event'."""
        cv = self.canvas
        if event is not None:
            if event.canvas != cv:
                raise RuntimeError
        self._bg = cv.copy_from_bbox(cv.figure.bbox)
        self._draw_animated()

    def add_artist(self, art):
        """
        Add an artist to be managed.

        Parameters
        ----------
        art : Artist

            The artist to be added.  Will be set to 'animated' (just
            to be safe).  *art* must be in the figure associated with
            the canvas this class is managing.

        """
        if art.figure != self.canvas.figure:
            raise RuntimeError
        art.set_animated(True)
        self._artists.append(art)

    def _draw_animated(self):
        """Draw all of the animated artists."""
        fig = self.canvas.figure
        for a in self._artists:
            fig.draw_artist(a)

    def update(self):
        """Update the screen with animated artists."""
        cv = self.canvas
        fig = cv.figure
        # paranoia in case we missed the draw event,
        if self._bg is None:
            self.on_draw(None)
        else:
            # restore the background
            cv.restore_region(self._bg)
            # draw all of the animated artists
            self._draw_animated()
            # update the GUI state
            cv.blit(fig.bbox)
        # let the GUI event loop process anything it has to do
        cv.flush_events()


def get_numbers(s: str) -> list[float]:
    nums = []
    for t in s.split():
        try:
            nums.append(float(t))
        except ValueError:
            pass
    return nums


def main():
    LEN = 100
    LINES_NUM = 3
    x = np.linspace(0, LEN, LEN)
    y = [np.zeros((1, LEN)).flatten() for i in range(0, LINES_NUM)]

    fig, ax = plt.subplots()

    lines = []
    for i in range(0, LINES_NUM):
        lines.append(ax.plot(x, y[i], animated=True)[0])

    # add a frame number
    fr_number = ax.annotate(
        "0",
        (0, 1),
        xycoords="axes fraction",
        xytext=(10, -10),
        textcoords="offset points",
        ha="left",
        va="top",
        animated=True,
    )
    bm = BlitManager(fig.canvas, lines + [fr_number])
    plt.ylim(-20000, 20000)
    # make sure our window is on the screen and drawn
    plt.show(block=False)
    plt.pause(.1)
    bm.update()

    frame_num = 0
    print("Setup complete, reading data...")
    for line in sys.stdin:
        print(line)
        frame_num += 1
        numbers = get_numbers(line)
        if len(numbers) != LINES_NUM:
            print(" - skip\n")
            continue

        # update the artists
        for i in range(0, LINES_NUM):
            y[i] = np.append(y[i][1:], numbers[i])
            lines[i].set_ydata(y[i])

        fr_number.set_text(f"frame: {frame_num}")

        # tell the blitting manager to do its thing
        if bm.closed:
            break
        else:
            bm.update()

    while (not bm.closed):
        bm.update()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
