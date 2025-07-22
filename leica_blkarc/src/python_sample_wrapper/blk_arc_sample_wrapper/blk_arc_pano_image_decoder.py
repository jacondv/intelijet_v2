import cv2
from dataclasses import dataclass
import math
import numpy as np


class BLK_ARC_PanoImageDecoder:
    WIDTH = 1456
    HEIGHT = 1098
    NUM_ROWS_WITH_METADATA = 10
    META_DATA_IN_BYTES = WIDTH * 6    # first 6 rows
    BLACK_LEVEL_CORRECTION_DATA_IN_BYTES = WIDTH * 4    # row 7-10 contain blacklevel correction data
    START_POS_IMAGE_PIXELS = (META_DATA_IN_BYTES + BLACK_LEVEL_CORRECTION_DATA_IN_BYTES)
    PADDING_BYTES_AT_THE_END = 16    # Final 16 bytes are used for padding

    @dataclass
    class Stats:
        sum: float = 0.0
        sum2: float = 0.0
        max: float = 0.0
        max2: float = 0.0

    @staticmethod
    def _clear_top_four_bits(img: np.ndarray) -> np.ndarray:
        return np.array([el & (np.uint16)(0x0FFF) for el in img])

    @classmethod
    def _calc_black_level_correction(cls, img: np.ndarray) -> float:
        rows_with_blk_level_data = 4
        sum: float = 0.0

        for r in range(rows_with_blk_level_data):
            # the very first 2 bytes per row contain FPGA info. ignore them
            local_start = cls.META_DATA_IN_BYTES + (r * cls.WIDTH) + 2
            local_end = local_start + cls.WIDTH - 2
            sum += np.sum(img[local_start:local_end], dtype=np.float32)

        # average it
        avg = sum / (rows_with_blk_level_data * (cls.WIDTH - 2))
        return avg

    @staticmethod
    def _apply_black_level_correction(img: np.ndarray, correction: float) -> np.ndarray:
        # if not applied, a hue shift towards green in the color image can be noticed.
        # black level corection can be omitted for grayscale conversion
        denom = (0x0FFF) - correction
        return np.array([255.0 * ((el - correction) / denom) for el in img]).clip(0.0, 255.0).astype(dtype=np.uint8)

    @staticmethod
    def _get_color_corrections(g: Stats, rb: Stats):
        a = (g.sum * rb.max2 - g.max * rb.sum2) / (rb.sum * rb.max2 - rb.sum2 * rb.max)
        b = (g.max * rb.sum - g.sum * rb.max) / (rb.sum * rb.max2 - rb.sum2 * rb.max)
        return [a, b]

    @staticmethod
    def _brightness_correction_grayscale(gray_image: np.ndarray):
        tmp = np.reshape(gray_image, (gray_image.shape[0] * gray_image.shape[1]))
        tmp = tmp.astype(np.float32) / 255.0

        # normalize to [0, 255]
        minValue = np.min(tmp)
        maxValue = np.max(tmp)
        if minValue != 0.0:
            tmp -= minValue
        tmp *= (255.0 / (maxValue - minValue))

        out = np.reshape(tmp, (gray_image.shape[0], gray_image.shape[1])).astype(np.uint8)
        return out

    @classmethod
    def _brightness_correction(cls, bgr_image: np.ndarray):
        tmp = np.reshape(bgr_image, (bgr_image.shape[0] * bgr_image.shape[1], bgr_image.shape[2]))
        tmp = tmp.astype(np.float32) / 255.0

        sums = np.sum(tmp, axis=0)
        squares = np.square(tmp)
        sums2 = np.sum(squares, axis=0)
        maxs = np.amax(tmp, axis=0)
        maxs2 = np.amax(squares, axis=0)

        b = cls.Stats(sums[0], sums2[0], maxs[0], maxs2[0])
        g = cls.Stats(sums[1], sums2[1], maxs[1], maxs2[1])
        r = cls.Stats(sums[2], sums2[2], maxs[2], maxs2[2])

        redParams = cls._get_color_corrections(g, r)
        blueParams = cls._get_color_corrections(g, b)
        tmp = np.array([[
            el[0] * blueParams[0] + el[0] * el[0] * blueParams[1], el[1],
            el[2] * redParams[0] + el[2] * el[2] * redParams[1]
        ] for el in tmp])

        # normalize to [0, 255]
        minValue = np.min(tmp)
        maxValue = np.max(tmp)
        if minValue != 0.0:
            tmp -= minValue
        tmp *= (255.0 / (maxValue - minValue))

        out = np.reshape(tmp, (bgr_image.shape[0], bgr_image.shape[1], bgr_image.shape[2])).astype(np.uint8)
        return out

    @staticmethod
    def _apply_gamma_correction(img: np.ndarray):
        # convert img to gray
        if (len(np.shape(img)) == 3):
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img

        # compute gamma = log(mid*255)/log(mean)
        mid = 0.3
        mean = np.mean(gray)
        gamma = math.log(mid * 255.0) / math.log(mean)

        # do gamma correction
        img_gamma = np.power(img, gamma).clip(0, 255).astype(np.uint8)
        return img_gamma

    @classmethod
    def _validateInput(cls, img_byte_array: bytearray) -> None:
        # first validate that the input is as expected
        if (len(img_byte_array) != (cls.WIDTH * cls.HEIGHT * 2) + cls.PADDING_BYTES_AT_THE_END):
            raise RuntimeError("Length of the provided image buffer is not as expected.")

        # expected offset = first 10 rows
        if (cls.START_POS_IMAGE_PIXELS != (cls.NUM_ROWS_WITH_METADATA * cls.WIDTH)):
            raise RuntimeError("Internal configuration error! Number of expected rows with metadata is wrong.")

        # the remaining number of bytes should be width*(height-padding)*2 .. or certainly be a multiple of the width
        if ((len(img_byte_array) - cls.START_POS_IMAGE_PIXELS - cls.PADDING_BYTES_AT_THE_END) % cls.WIDTH != 0):
            raise RuntimeError("Unexpected image width.")

    @classmethod
    def _commonDecodingStart(cls, img_byte_array: bytearray, applyBlackLevelCorrection: bool = True) -> np.ndarray:
        cls._validateInput(img_byte_array)

        # load buffer into np array with format uint16
        bayer_image_raw: np.ndarray = np.frombuffer(img_byte_array, np.uint16)

        # clear dirty bits --> all bits, also the first 10 rows
        bayer_image_cleaned = cls._clear_top_four_bits(bayer_image_raw)

        # black level correction
        if applyBlackLevelCorrection:
            correction = cls._calc_black_level_correction(bayer_image_cleaned)
            pixels = bayer_image_cleaned[cls.START_POS_IMAGE_PIXELS:-8]
            pixels_corrected = cls._apply_black_level_correction(pixels, correction)
        else:
            pixels_corrected = bayer_image_cleaned[cls.START_POS_IMAGE_PIXELS:-8]

        # reshape
        pixels_reshaped = pixels_corrected.reshape((cls.HEIGHT - cls.NUM_ROWS_WITH_METADATA, cls.WIDTH))
        return pixels_reshaped

    @classmethod
    def decodeToGrayscale(cls, img_byte_array: bytearray) -> np.ndarray:
        pixels_reshaped = cls._commonDecodingStart(img_byte_array, applyBlackLevelCorrection=False)

        gray_image = cv2.cvtColor(pixels_reshaped, cv2.COLOR_BAYER_BG2GRAY)

        brightness_corrected = cls._brightness_correction_grayscale(gray_image)

        grayscale_img = cls._apply_gamma_correction(brightness_corrected)
        return grayscale_img

    @classmethod
    def decodeToColor(cls, img_byte_array: bytearray) -> np.ndarray:
        pixels_reshaped = cls._commonDecodingStart(img_byte_array, applyBlackLevelCorrection=True)

        # convert to BGR
        bgr_image = cv2.cvtColor(pixels_reshaped, cv2.COLOR_BAYER_BG2BGR)

        # correct for brightness and green channel dominance
        brightness_corrected = cls._brightness_correction(bgr_image)

        # apply gamma curve
        color_img = cls._apply_gamma_correction(brightness_corrected)
        return color_img
