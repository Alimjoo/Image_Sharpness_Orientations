# Image_Sharpness_Orientations

C++17/OpenCV library for registering a reference product image and measuring the
corresponding dark or light parts in subsequent images, plus independent outer
projection-edge detection with ellipse fitting and nine-point brightness measurement. The public interface follows
`Alg_CH`: an exported `CH_...` class, `CHImage`/contour aliases, parameter/result
structures, integer return codes, and Chinese API comments.

The four operations (`RegisterBase`, `Detect`, `DetectOuterEdge`, and `DetectBrightness`) are declared in
`projs/Image_Sharpness_Orientations/headers/image_sharpness_orientations.h`.
Their types are in the adjacent `image_sharpness_orientations_structs.h`.

## Usage

Keep one class instance for each product design. Registration stores its own
contours and parameters in the instance's memory; the source image and returned
result may then be released.

Registration and matching use separate output structures:

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
return explicit error codes. `Detect()` requires a successfully registered base;
`DetectOuterEdge()` and `DetectBrightness()` work independently without registration.
A failed re-registration preserves the previous base. Instances are noncopyable;
do not register and detect concurrently on the same instance.

## Parameters

Registration and matching use three settings:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `detect_dark_area` | `true` | Dark marks on a bright product; `false` selects white/light marks on a dark product. |
| `threshold` | `-1` | Otsu automatic threshold; 0–255 selects a fixed grayscale threshold. Dark pixels are <= threshold; light pixels are > threshold. |
| `min_area` | `100.0` | Minimum outer-contour area in pixels squared; must be finite and positive. |

These settings are saved by `RegisterBase()` and reused by `Detect()`. Register
again to change them. Private segmentation/measurement constants include 3x3
Gaussian smoothing, maximum area ratio 0.25, a 2-pixel contour band, orientation
anisotropy 0.08, and a 5-pixel local-background dilation radius. Matching allows
a center residual up to 0.12 in the normalized product frame. Area and shape
differences contribute bounded costs; they no longer impose rejection thresholds.

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
4. Normalize positions using the product silhouette's center and covariance.
   This accounts for resolution, nonuniform scale, and shear without modifying
   the measured contours. Search the remaining rotation within +/-90 degrees
   using the complete mark layout.
5. Match all parts jointly with a minimum-cost one-to-one assignment. Position
   and soft left/right and above/below order dominate the cost; area, contour
   compactness, and moment anisotropy are secondary factors. Almost equal
   coordinates are treated as order ties. Missing/extra parts can remain
   unmatched, even when the two images contain the same number of parts.
6. Return matching parts in base-ID order, followed by unmatched parts with ID -1.
   Missing base IDs are listed separately. `ISO_NoShapes` lists all base IDs as
   missing when a valid detection image contains no accepted contours.

`all_matched` requires every base part and every current part to participate.
`match_cost` is an assignment cost, not a probability, calibrated confidence,
or amount of deformation. An unmatched contour does not necessarily represent
a newly added part; the overlay labels it `unmatched`.

The matcher assumes the same product layout with separated marks of the opposite
polarity and a usable product silhouette. It tolerates moderate perspective and
local deformation when centers retain their relative layout after normalization;
it does not estimate a full projective or nonrigid warp. It does not split touching
marks, recover clipped parts, or resolve arbitrary 180-degree/symmetric poses.
Large position changes or severe silhouette changes can still leave parts
unmatched. Appearance changes alone are deliberately weak evidence against a
match, so this is not a classifier for rejecting different product designs.
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

Matching normalization is internal: returned centers, contours, areas, and
orientations still describe the original input image. This preserves the
differences needed for subsequent deformation analysis. Across different
resolutions or camera poses, account for image scale/registration before
interpreting raw pixel or area differences as physical deformation. The overall
`angle` remains a least-squares rotation of matched centers in original image
coordinates; it does not describe the complete distortion.

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

## Independent outer-edge detection

`DetectOuterEdge()` measures the large boundary around the entire projection.
It runs without a base image or internal marks and does not read or modify the
registered base or its parameters. Use the separate result and parameter types:

```cpp
CH_Image_Sharpness_Orientations detector;
Image_Sharpness_Orientations_Outer_Edge_Params params;
params.detect_bright_area = true; // bright projection on a dark background
Image_Sharpness_Orientations_Outer_Edge_Result edge;
const int status = detector.DetectOuterEdge(image, edge, params);
if (status == ISO_OK) {
    const double major_diameter_px = 2.0 * edge.a;
    const double minor_diameter_px = 2.0 * edge.b;
    // Use edge.axis_ratio, edge.fit_error, and edge.sharpness with your
    // product's acceptance limits. Check edge.orientation_valid for angles.
}
```

The parameters are independent of the mark-detection parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `detect_bright_area` | `true` | Select the bright projection; `false` selects a dark projection on a bright background. |
| `threshold` | `-1` | Otsu threshold; 0–255 selects a fixed threshold. Bright pixels are > threshold; dark pixels are <= threshold. |
| `min_area` | `100.0` | Minimum area enclosed by the outer contour, in pixels squared; finite and positive. |

The detector smooths only for segmentation, selects the largest external contour,
and fits an ellipse to its dense boundary points with OpenCV `fitEllipse`.
Internal marks and holes do not contribute fitting points. Unlike part extraction,
there is no maximum area ratio, so the projection can occupy most of the image.

| Result | Meaning |
| --- | --- |
| `image_size`, `contour`, `bounds`, `area` | Input size and actual segmented boundary geometry; area includes internal marks and holes. |
| `center` | Fitted ellipse center in input-image pixel coordinates. |
| `a`, `b` | Major and minor **semi-axes** in pixels, `a >= b > 0`; diameters are `2*a` and `2*b`. |
| `orientation` | Major-axis direction, [0, 180) degrees clockwise in image coordinates. |
| `orientation_valid` | False for a nearly circular ellipse; the remaining measurements are still usable. |
| `axis_ratio` | `b/a`, in (0, 1]; a circle is near 1. |
| `fit_error` | Dimensionless RMS error of the normalized ellipse radius; lower is a closer fit. |
| `mean_brightness` | Mean original grayscale intensity inside the entire outer contour, including internal marks, 0–255. |
| `sharpness` | Unscaled Sobel/Tenengrad energy around the **actual detected boundary**, using the same contour-band formula as the part measurements. |
| `contrast` | Absolute difference of local grayscale means on the inside and outside of the outer edge, 0–255; internal mark edges are excluded. |

For `fit_error`, points are translated to the fitted center and rotated into the
major/minor-axis frame `(u, v)`. The reported value is
`sqrt(mean((sqrt((u/a)^2 + (v/b)^2) - 1)^2))`; it is not a pixel distance.
An irregular boundary can have an axis ratio near 1, so inspect both the ratio and
fit error when judging whether a projection is circular. The angle is flagged
unreliable when `(a*a - b*b)/(a*a + b*b) < 0.08`; its fitted value is still returned
so the ellipse can be drawn. To draw it, construct an OpenCV `RotatedRect` using
`center`, size `(2*a, 2*b)`, and `orientation`.

Sharpness uses a contour band with half-width 2 pixels on the original grayscale
image. Contrast uses the inner and outer portions of a band with half-width 5
pixels. These are outer-edge quality measurements, separate from the internal
marks measured by `RegisterBase()` and `Detect()`.

The full projection must be visible with background around it. A largest contour
touching the image boundary returns `ISO_OuterEdgeClipped`; the detector does not
substitute a smaller object. Uniform images or no qualifying contour return
`ISO_NoShapes`; an invalid ellipse fit returns `ISO_EllipseFitFailed`. All failure
paths clear the output. Multiple objects, poor thresholding, or features touching
the projection boundary can change which contour is measured; the routine does
not reconstruct a cropped or broken outline.

`ISO_OK` reports successful measurement. Product acceptance limits and physical
pixel-to-length calibration belong to the caller. Compare sharpness under the
same exposure, contrast, resolution, and capture conditions.

## Nine-point brightness measurement

`DetectBrightness()` returns **nine separate brightness measurements**, one mean
per circular sampling disk. There is no combined average of the nine points.
It does not use or modify a registered base. All coordinates and distances refer
to the original input image, with x increasing rightward and y increasing downward.

```cpp
CH_Image_Sharpness_Orientations detector;
Image_Sharpness_Orientations_Brightness_Params params;
params.mode = Image_Sharpness_Orientations_Brightness_Mode::AvoidBlack;
params.inset_px = 30;
params.spot_diameter_px = 21;
params.black_threshold = 80;  // <=80 is unavailable; -1 selects Otsu
params.search_radius_px = 60;
Image_Sharpness_Orientations_Brightness_Result result;
const int status = detector.DetectBrightness(image, result, params);
if (status == ISO_OK || status == ISO_PartialMeasurement) {
    for (const auto& point : result.points) {
        if (!point.valid) continue; // Inspect point.status for the reason.
        // point.mean_brightness: this disk's mean grayscale value
        // point.requested_center: original automatic or manual position
        // point.center: actual measurement position; point.moved reports relocation
    }
}
```

| Mode | Placement and sampling |
| --- | --- |
| `Auto` | Fit the projection ellipse, place its center plus eight peripheral points, and measure each disk as placed, including dark pixels. |
| `AvoidBlack` | Start with the same nine points. Keep a disk only if every sampled pixel is inside the actual projection contour and above the black threshold. Otherwise find the nearest valid integer-pixel center within the search radius. |
| `Manual` | Measure exactly the nine supplied centers, in input order. No ellipse extraction, automatic movement, or black-pixel exclusion. |

Automatic IDs are fixed in image directions: **1 center; 2 top; 3 top-right;
4 right; 5 bottom-right; 6 bottom; 7 bottom-left; 8 left; 9 top-left**.
Eight rays, 45 degrees apart, intersect the fitted rotated ellipse. Each peripheral
center moves inward along its ray by `inset_px`. The inset is the distance to the
**disk center**, not the disk rim; use an inset larger than half the diameter to
leave a margin. It must be smaller than each ray's boundary distance. The center
point is unaffected by the inset. This numbering also remains stable for circles
whose fitted principal-axis angle is unreliable.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `mode` | `Auto` | One of the three placement modes above. |
| `inset_px` | `20` | Nonnegative inward distance from fitted boundary to peripheral sample center. Used by `Auto` and `AvoidBlack`. |
| `spot_diameter_px` | `11` | Positive integer disk diameter; radius is diameter/2.0, including for even diameters. Used in all modes. |
| `black_threshold` | `-1` | `AvoidBlack` only: grayscale <= threshold is unavailable. -1 computes Otsu on the original grayscale image; otherwise 0–255. |
| `search_radius_px` | `50` | `AvoidBlack` only: maximum Euclidean distance from original center to replacement; 0 disables movement. |
| `points` | empty | `Manual` requires exactly nine finite `CHPointF` coordinates. Ignored in automatic modes. |
| `outer_edge` | default outer-edge parameters | Contour polarity, threshold, and minimum area for ellipse fitting. Ignored in `Manual`. |

For manual operation, set `params.mode` to `Manual` and fill `params.points` with
nine image coordinates. Uniform images and images without a detectable projection
are supported in this mode. Manual points can be fractional. The disk includes
all integer pixel centers whose distance from the supplied center is <= diameter/2.
The whole disk must fit within image coordinates `[0, width-1] x [0, height-1]`;
it is never cropped to obtain a reading. A diameter-1 disk at some fractional
positions contains no pixel centers and reports `NoPixels`.

Avoidance uses the original, unsmoothed grayscale image to identify dark pixels,
including isolated dark pixels at the sampling disk's rim. It intersects this
bright-pixel mask with the actual projection silhouette, so an imperfect ellipse
cannot authorize sampling outside the projection. A local distance transform
screens candidate centers, then the exact sampling disk is checked again. Search
is independent for each point; the nearest valid integer center is selected, with
ties resolved by smaller y then smaller x. An unchanged valid center retains its
original fractional coordinates. No partial disk or dark-pixel removal is used
to make an otherwise invalid reading pass.

`result.points` always contains nine entries after a completed or partial
measurement, with stable IDs, `requested_center`, actual `center`, `moved`, `valid`,
`status`, `pixel_count`, and `mean_brightness`. The result also reports `valid_count`,
`all_valid`, disk diameter, effective black threshold, and the fitted ellipse
geometry used for automatic placement. Manual results have `ellipse_valid=false`.

`ISO_OK` means all nine disks were measured. `ISO_PartialMeasurement` preserves all
nine entries, including any valid readings; an invalid entry reports
`OutsideImage`, `NoPixels`, or `NoBrightArea`. Its brightness and pixel count stay
zero, and an unsuccessful relocation retains the original center with
`moved=false`. Check `valid` to distinguish an invalid reading from a valid black
disk with brightness zero. Other errors, such as malformed parameters, unsupported
images, or inability to fit the outer boundary in automatic modes, clear the result.

Values are original grayscale intensity means in **0–255**, with BGR/BGRA converted
using OpenCV's grayscale conversion. They are camera brightness readings; physical
illuminance in lux or luminance in cd/m² requires a separate camera/exposure
calibration. The API does not infer those units or apply product acceptance limits.

## Source organization

`projs/Image_Sharpness_Orientations` owns the public headers, exported API,
instance state, and packaging. Its `.cpp` delegates the algorithm work to the
internal static library in `projs/Image_Sharpness_Orientations_Core`:

- `shape_extraction.cpp`: parameter validation, segmentation, product-frame
  normalization, contour extraction, and principal-axis measurements.
- `quality_measurement.cpp`: foreground brightness, local contrast, and
  Sobel/Tenengrad sharpness for parts and the outer projection edge.
- `outer_edge.cpp`: independent projection segmentation and ellipse fitting.
- `brightness_measurement.cpp`: nine-point placement, full-disk sampling, and
  nearest valid placement for black-area avoidance.
- `assignment.cpp`: minimum-cost one-to-one assignment.
- `layout_matching.cpp`: layout descriptors, rotation search, shape matching,
  and relative-angle results.

The core library is linked privately into the final API library. Its headers
and static archive are internal build artifacts; consumers continue to use the
same public headers and shared library.

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
scale, nonuniform resizing, shear, a vertical layout, moderate perspective,
and missing middle arcs. They check each ID against its transformed contour,
preserve raw deformed areas, and verify that changed/swapped appearances do not
override spatial identity. Other checks cover known angles and areas, reduced
sharpness after blur, missing/extra parts,
known foreground brightness and local contrast, exclusion of holes and nearby
parts, quadratic Tenengrad response to edge contrast, dark/light selection with
automatic and fixed thresholds, input formats,
invalid parameters, independent instances, and failed re-registration retaining
the existing base. Outer-edge checks cover circles, rotated ellipses with known
centers and semi-axes, blur, contrast scaling, internal-mark exclusion, bright/dark
projection polarity, color and non-contiguous inputs, clipped projections,
invalid parameters, output clearing, and preservation of the registered base.
Nine-point tests cover rotated-ellipse placement, pixel insets, individual known
brightness values, manual coordinates, full-disk overlap with isolated dark pixels,
nearest replacements, bounded failed searches, irregular outlines, exclusion of
bright background outside the projection, formats, and partial measurement states.

## Run on image files

To label and measure the nine brightness disks:

```sh
cmake --build build --config Debug
./build/Image_Sharpness_Orientations_test --brightness "/path/to/image.bmp" "test_output/brightness" --mode auto --inset 30 --diameter 21
./build/Image_Sharpness_Orientations_test --brightness "/path/to/image.bmp" "test_output/brightness_avoid" --mode avoid-black --inset 30 --diameter 21 --black-threshold 80 --search-radius 60
./build/Image_Sharpness_Orientations_test --brightness "/path/to/image.bmp" "test_output/brightness_manual" --mode manual --diameter 21 --points "points.txt"
```

For manual mode, `points.txt` contains exactly nine whitespace-separated `x y`
pairs, normally one pair per line. `--threshold N` sets the outer-boundary
segmentation threshold; `--black-threshold N` independently controls which
original pixels are excluded in avoidance mode. `--dark` selects a dark projection
for the outer-boundary fit. Both thresholds default to Otsu.
`--black-threshold` and `--search-radius` are accepted only with
`--mode avoid-black`; passing either in `auto` or `manual` mode is a command-line
error. The corresponding C++ parameter fields are ignored outside `AvoidBlack`.

The runner writes `brightness.png` with numbered disks and individual values:
green means valid, red invalid, and yellow crosses/arrows show original positions
and their replacements. `brightness.csv` has one row per point, including both
positions, movement/validity flags, reason, diameter, pixel count, and brightness.
Invalid brightness cells are blank. Exit code 0 means all nine readings are valid;
partial measurements return 1 and still save the overlay and all nine CSV rows.

For the independent outer-edge test, build and run from the repository directory:

```sh
cmake --build build --config Debug
./build/Image_Sharpness_Orientations_test --outer-edge "/path/to/image.bmp" "test_output/outer_edge"
```

On Windows with Visual Studio, use
`build\Debug\Image_Sharpness_Orientations_test.exe` (or the corresponding Release
executable). No base image is needed. The default is a bright projection on a dark
background; append `--dark` for the opposite polarity. Optional `--threshold 100`
sets a fixed threshold (default `-1` means Otsu), and `--min-area 1000` sets the
minimum area in pixels squared.

The runner prints the ellipse and quality measurements and saves `outer_edge.png`
(green measured boundary, red fitted ellipse, cyan center) and `outer_edge.csv`.
The overlay may be resized for viewing; all numeric measurements refer to the
original image. `a` and `b` are semi-axes in pixels. Exit code 0 means measurement
succeeded, not that the projection passed customer limits. Failure prints an
error and leaves existing output files unchanged. Running the executable without
arguments runs the automated tests.

For the existing internal-mark registration/matching test:

```text
build/Release/Image_Sharpness_Orientations_test.exe --images BASE_IMAGE OUTPUT_DIR IMAGE [IMAGE ...]
```

The runner uses the default dark-area settings. It writes `base.png`, a numbered contour overlay for each current image,
`summary.csv`, and `measurements.csv`. Both CSV files include a `stage` column
(`register` or `detect`); detection-only matching fields are blank for registration.
Unmatched contours are red and labeled `unmatched`.
It supports Windows Unicode input paths.
Exit status is 0 only when every current image has a complete match; 1 also
indicates a successfully processed series containing missing/extra parts, so
inspect the CSV status column to distinguish processing errors.

The layout matcher was checked on the supplied 5472 x 3648 base images against
`fsafds.bmp` (1501 x 1501). Both pairs now retain all eight correct IDs, including
the repeated arcs. Results are under `test_output/layout/` (base ending
`65418453`) and `test_output/layout_original_base/` (base ending `64834661`).
`test_output/layout_series/` contains the latter base plus the other 18
same-product images, with all eight IDs matched in every detection. The CSVs
include per-part `mean_brightness`, Tenengrad `sharpness`, and `contrast`.

Earlier validation on the mixed 27-image series is saved under `test_output/`.
These older snapshots predate the new matcher, quality columns, and Tenengrad
score; use the `layout*` folders for the current measurements:

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
