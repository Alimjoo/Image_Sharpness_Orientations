# Image_Sharpness_Orientations

C++17/OpenCV library for registering a reference product image and measuring the
corresponding dark or light parts in subsequent images. The public interface follows
`Alg_CH`: an exported `CH_...` class, `CHImage`/contour aliases, parameter/result
structures, integer return codes, and Chinese API comments.

The two operations are declared in
`projs/Image_Sharpness_Orientations/headers/image_sharpness_orientations.h`.
Their types are in the adjacent `image_sharpness_orientations_structs.h`.

## Usage

Keep one class instance for each product design. Registration stores its own
contours and parameters in the instance's memory; the source image and returned
result may then be released.

The two operations use separate output structures:

- `Image_Sharpness_Orientations_Register_Result`: base image size and numbered
  shapes, including their geometry and quality measurements.
- `Image_Sharpness_Orientations_Detect_Result`: current image size and shapes,
  plus matching counts, missing base IDs, and overall relative rotation.

Both use the same per-shape measurement structure. Its `matched`, `angle`,
`angle_valid`, and `match_cost` fields are detection-only and remain at their
initial defaults in registration results. Registration supplies each shape's
absolute principal-axis `orientation` and `orientation_valid` instead.

```cpp
#include "image_sharpness_orientations.h"

int inspect(const CHImage& base_image, const CHImage& current_image,
    Image_Sharpness_Orientations_Detect_Result& current_result) {
    CH_Image_Sharpness_Orientations detector;
    Image_Sharpness_Orientations_Params params;
    params.detect_dark_area = true; // false for white/light marks on a dark product
    Image_Sharpness_Orientations_Register_Result base_result;

    int ret = detector.RegisterBase(base_image, base_result, params);
    if (ret != ISO_OK) return ret;

    // Reuse this detector for further images of the same design.
    ret = detector.Detect(current_image, current_result);
    if (ret != ISO_OK) return ret;

    // ISO_OK means processing succeeded. Inspect all_matched, missing_ids,
    // and each shape's matched/angle_valid fields before using the result.
    return ISO_OK;
}
```

Inputs are 2D 8-bit grayscale, BGR, or BGRA images. Empty and unsupported images
return explicit error codes. Detection requires a successfully registered base.
A failed re-registration preserves the previous base. Instances are noncopyable;
do not register and detect concurrently on the same instance.

## Parameters

Only three settings are public:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `detect_dark_area` | `true` | Dark marks on a bright product; `false` selects white/light marks on a dark product. |
| `threshold` | `-1` | Otsu automatic threshold; 0–255 selects a fixed grayscale threshold. Dark pixels are <= threshold; light pixels are > threshold. |
| `min_area` | `100.0` | Minimum outer-contour area in pixels squared; must be finite and positive. |

These settings are saved by `RegisterBase()` and reused by `Detect()`. Register
again to change them. The other settings retain their previous values as private
implementation constants: 3x3 Gaussian smoothing, maximum area ratio 0.25,
2-pixel contour band, match distance 0.12, area factor 2.0, shape distance 1.0,
and orientation anisotropy 0.08. Local background sampling uses a fixed
5-pixel dilation radius.

The simplified parameter struct replaces the previous tuning fields. Quality
results now use float `mean_brightness`, `sharpness`, and `contrast` fields.
The separate registration and detection result types replace the former shared
`Image_Sharpness_Orientations_Result`. Update callers and recompile consumers
with the updated public headers and library.

## Extraction and matching

1. Smooth only for segmentation, then threshold the selected dark or light pixels
   (Otsu by default).
2. Identify the largest product silhouette of the opposite polarity: bright for
   dark marks, dark for light marks. Extract selected outer contours inside it,
   ignoring inner holes, small regions, and regions touching the image border.
   Each selected connected region is one part.
3. Number base parts from 1 in increasing center-x order (then center-y for ties).
4. Match nearby centers after correcting product position and scale, with area,
   contour compactness, and moment anisotropy checks. Greedy matching uses each
   base part and detected part at most once. A second pass refines rotation from
   the initially matched centers.
5. Return matching parts in base-ID order, followed by extra parts with ID -1.
   Missing base IDs are listed separately. `ISO_NoShapes` lists all base IDs as
   missing when a valid detection image contains no accepted contours.

`all_matched` requires every base part and every current part to participate.
`match_cost` is a comparison cost, not a probability or a calibrated confidence.

This initial implementation assumes a single product with separated marks of
the opposite polarity and approximately similar orientation. It does not split touching marks,
recover clipped parts, correct perspective, or search arbitrary 180-degree poses.
Faint/broken strokes can change the connected-part count; tune `threshold`,
`min_area`, and `detect_dark_area` for the product.

## Measurements

| Field | Meaning |
| --- | --- |
| `contour` | Outer contour in original image coordinates; no inner-hole contours. |
| `area` | Area enclosed by that outer contour, in pixels squared, including holes. |
| `orientation` | Undirected principal axis, [0, 180) degrees, clockwise in image coordinates. |
| Per-part `angle` | Difference from the corresponding base principal axis, [-90, 90) degrees. |
| Detection result `angle` | Overall rotation from matched centers, [-180, 180) degrees; a single matched part uses its principal axis. |
| `mean_brightness` | Mean grayscale intensity of this connected foreground part, 0–255; excludes holes and other parts. |
| `sharpness` | Sobel/Tenengrad mean gradient energy, `mean(Gx² + Gy²)`, in a narrow band around the outer contour. |
| `contrast` | Absolute difference between foreground and local-background mean brightness, 0–255. |

Always inspect `orientation_valid` and `angle_valid`. Nearly circular shapes
have unreliable principal axes. Perspective can change per-part principal-axis
angles differently from the whole-pattern rotation.

All three quality measurements use the original grayscale pixels, not the
smoothed segmentation image. Foreground brightness uses only the current
connected part, even when another part lies inside its hole.

Local background is sampled within a 5-pixel dilation around that part, including
background along hole boundaries. Every foreground region and pixels outside
the product silhouette are excluded. Contrast is
`abs(mean_foreground - mean_local_background)`, so both dark and light parts
produce nonnegative contrast. If no local background pixels are available,
contrast is 0.

Sharpness uses unscaled 3x3 Sobel derivatives (`CV_32F`, scale 1), following
the Tenengrad energy formula used in `Alg_CH`. The mean of `Gx² + Gy²`
is taken over the existing outer-contour band (half-width 2 pixels), without
a gradient cutoff. It is an energy score, not a 0–255 brightness value.
This replaces the earlier average normalized gradient magnitude; old and new
sharpness scores are not directly comparable.

Higher energy generally indicates sharper edges under consistent capture
conditions. Contrast, exposure, resolution, and the sampling band also affect
the score. This is a relative sharpness measure, not an MTF measurement.
The formula matches `Alg_CH`; the sampling region here is the shape's contour
band rather than an OCR character crop.

The geometry and filtering primitives are documented by OpenCV:
[contours and moments](https://docs.opencv.org/4.12.0/d3/dc0/group__imgproc__shape.html),
[Sobel filtering](https://docs.opencv.org/4.12.0/d4/d86/group__imgproc__filter.html).

## Build and test

Requires CMake 3.18+, a C++17 compiler, and OpenCV 4.x (tested with 4.12.0).
The library uses `core` and `imgproc`; the test/image runner also uses `imgcodecs`.

```sh
cmake -S . -B build
cmake --build build --config Release
cmake -E chdir build ctest -C Release --output-on-failure
cmake --build build --config Debug
cmake -E chdir build ctest -C Debug --output-on-failure
```

On Windows, the default OpenCV location follows `Alg_CH`:
`%USERPROFILE%/Documents/opencv/build`. Override it with
`-DOpenCV_DIR=/path/to/opencv/config`. A new Visual Studio build directory can
use `-G "Visual Studio 17 2022" -A x64`; keep the same generator settings when
reusing an existing directory. For a single-configuration generator, specify
`-DCMAKE_BUILD_TYPE=Release`. Set `-DBUILD_TESTING=OFF` for only the library.

The tests cover eight-part extraction, stable IDs under translation/rotation/
scale, known angles and areas, reduced sharpness after blur, missing/extra parts,
known foreground brightness and local contrast, exclusion of holes and nearby
parts, quadratic Tenengrad response to edge contrast, dark/light selection with
automatic and fixed thresholds, input formats,
invalid parameters, independent instances, and failed re-registration retaining
the existing base.

## Run on image files

```text
build/Release/Image_Sharpness_Orientations_test.exe --images BASE_IMAGE OUTPUT_DIR IMAGE [IMAGE ...]
```

The runner uses the default dark-area settings. It writes `base.png`, a numbered contour overlay for each current image,
`summary.csv`, and `measurements.csv`. Both CSV files include a `stage` column
(`register` or `detect`); detection-only matching fields are blank for registration.
It supports Windows Unicode input paths.
Exit status is 0 only when every current image has a complete match; 1 also
indicates a successfully processed series containing missing/extra parts, so
inspect the CSV status column to distinguish processing errors.

The supplied same-product folder is tested under `test_output/same/`: one base
plus 18 detections, with all eight IDs matched in every detection.
The refreshed `measurements.csv` includes per-part `mean_brightness`,
Tenengrad `sharpness`, and `contrast`.

Earlier validation on the mixed 27-image series is saved under `test_output/`.
These older snapshots predate the new quality columns and Tenengrad score;
use `same/` for the current measurements:

- `footprint/`: one base plus 18 current images; all eight base IDs matched in
  every current image.
- `vehicle/`: a separate base plus seven current images; faint, broken, or
  cropped strokes produce incomplete matches and are recorded in the reports.
- `different_design/`: comparing the vehicle against the footprint base found
  no matching parts.

The automated motion tests establish known correspondence and angle accuracy.
The real-image results are initial extraction/matching checks, not manually
calibrated metrology or ground-truth sharpness measurements.

## Package outputs

Each build writes to
`build/projs/Image_Sharpness_Orientations/Image_Sharpness_Orientations_<Config>/`
and collects public files in
`build/projs/Image_Sharpness_Orientations/Image_Sharpness_Orientations_Public_Release/`.

The public package contains both headers, the library, and on Windows the import
library. OpenCV DLLs are not copied into the library output or public package.
Only the local test executable receives OpenCV runtime copies. Debug libraries
use the `d` suffix so Debug and Release coexist.

Consumers supply their own compatible OpenCV development headers, libraries,
and runtime DLLs. Include the package directory and OpenCV include directory,
link the corresponding import libraries, and make the matching runtime DLLs
available to the consuming executable.
