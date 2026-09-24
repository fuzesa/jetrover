import numpy as np

from jetrover_sorting.display import Overlay, Texts, render


def frame():
    img = np.zeros((360, 640, 3), np.uint8)
    img[:, :320] = (200, 0, 0)          # left half red (RGB)
    return img


def test_render_keeps_size_and_converts_to_bgr():
    out = render(frame(), Overlay(), Texts(), mirror=False)
    assert out.shape == (360, 640, 3)
    assert tuple(out[300, 10]) == (0, 0, 200)                  # red, now in BGR order


def test_mirror_flips_the_image():
    out = render(frame(), Overlay(), Texts(), mirror=True)
    assert tuple(out[300, 10]) == (0, 0, 0) and tuple(out[300, 630]) == (0, 0, 200)


def test_following_draws_circle_on_the_mirrored_side_and_a_bar():
    ov = Overlay('following', 'green', x=100, y=180, radius=30, z=0.21, progress=0.5)
    blank = np.zeros((360, 640, 3), np.uint8)
    out = render(blank, ov, Texts(), mirror=True)
    ring = out[180, 640 - 1 - 100 - 34]                        # circle edge, mirrored x
    assert ring.any()
    assert not out[180, 100 - 34].any()                        # nothing at the unmirrored spot
    assert out[360 - 20, int(640 * 0.3)].any()                 # bar filled at 30 % width
    assert not out[360 - 20, int(640 * 0.75)].any() or tuple(out[360 - 20, int(640 * 0.75)]) == (0, 0, 0)


def test_every_mode_renders_with_romanian_texts():
    t = Texts('Arata-mi un cub!', 'Vad un cub {color}', 'Tine-l nemiscat...', 'L-am prins!',
              'Cubul {color} merge in cutia lui', {'red': 'rosu', 'green': 'verde', 'blue': 'albastru'})
    for mode in ('searching', 'following', 'grabbing', 'placing'):
        out = render(frame(), Overlay(mode, 'blue', 320, 180, 25, None, 0.9), t)
        assert out.shape == (360, 640, 3)
