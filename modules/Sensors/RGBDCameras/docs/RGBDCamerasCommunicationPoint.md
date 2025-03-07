@ingroup group_rgbdcamera_communication_point

This is the documentation for the API of the RGBDCamerasCommunciationPoint. It includes a description of the messages to send and the expected replies.

- [RGBD Cameras Communication Point {#rgbdcameras\_communication\_point}](#rgbd-cameras-communication-point-rgbdcameras_communication_point)
  - [Notes](#notes)
  - [Available API calls](#available-api-calls)
    - [Start Frame Stream](#start-frame-stream)
    - [Frame Streamer](#frame-streamer)
    - [Set Profile](#set-profile)
    - [Get Status](#get-status)

### Notes

This communication point inherits from the [CamerasCommunciationPoint](@ref cameras_communication_point) and as such it is recommended to check it to understand all the inherited functionalities and calls.

Some return types of this communication point include crf::expected<T>. To understand how these messages are parsed please refer to the correspondent documentation, [Expected](@ref error_handler)

### Available API calls

#### Start Frame Stream

This method has been overwritten from the cameras and the way to call it has slight variations:

```json
{
    "command" : "startFrameStream",
    "image_profile" : {
        "resolution" : {
            "width" : 640,
            "height" : 360
        },
        "framerate" : 30,
    },
    "encoding_quality" : 5,
    "encoding_format" : 1,
    "depth_profile" : {
        "resolution" : {
            "width" : 640,
            "height" : 360
        },
        "framerate" : 30,
    },
    "depth_format" : 2,
    "point cloud_format" : 2,
    "point cloud_subsampling" : 2.5
}
```

Since the image profile and the encoding are explained in the cameras module we won't go into detail here.

The only mandatory fields are image resolution, encoding quality, and encoding format, the rest are optional. If no other fields are provided only the image is activated.

If the fields depth profile and depth format are present then depth will be activated.

Depth format is an enum with the following structure:

- 1 cvMat
- 2 LZ4

Where cvMat takes the OpenCV matrix and writes it directly into a string of bytes and LZ4 uses the compression algorithm of the same name to compress this same string of bytes. The enum decides if the LZ4 compression is done or skipped.

If the fields point cloud format and point cloud subsampling are provided then the point clouds are activated.

point cloud format is an enum that has the following values:

- 1 PLY
- 2 LZ4

The format works the same way as the depth, the point cloud gets serialized into PLY. If LZ4 is selected then it gets compressed and sent, otherwise, the raw point cloud is sent.

The subsampling is used to reduce the size of the point cloud by reducing the number of samples in it. The value provided represents the factor by which this subsampling is done.

In case of errors, the response will follow a JSONError type with a string explaining the error.
If the request was successful a JSONReply with a boolean will be received and if it's true the streamer will start.

#### Frame Streamer

Once started, the frame streamer is responsible to send the frames to the client respecting the framerate requested.

The frame will be sent using an [RGBDFramePacket]{reference_missing} and the selected encoding.

#### Set Profile

Setting the profile has also been changed to add depth and point cloud parameters.

```json
{
    "command" : "setProfile",
    "image_profile" : {
        "resolution" : { "width" : 640, "height" : 360 },
        "framerate" : [15]  // IMPORTANT!!! IT'S AN ARRAY OF SIZE 1
    },
    "depth_profile" : {
        "resolution" : {
            "width" : 640,
            "height" : 360
        },
        "framerate" : 30,
    },
    "point cloud_subsampling" : 2.5
}
```

Changing formats while the stream is running is not permitted, to achieve this stop and start a new frame streamer.

The parameters depth profile and point cloud subsampling are optional, if they are present they will activate depth and point clouds respectively, if not present they will deactivate them.

In case of errors, the response will follow a JSONError type with a string explaining the error. If the request was successful a JSONReply with a boolean will be received.

#### Get Status

As it's standard in all communication points a get status function is implemented that can be configured with a streamer. Check [PriorityAccessCommunicationPoint]{reference_missing} for more information.

The status returned by this communication point will show the next fields.

```json
{
    "status" : "initialized",
    "property" : {
        "zoom" : {
            "value" : 3,
            "code" : 200,
            "detail" : 0
        },
        "iso" : {
            "code" : 405,
            "detail" : 0
        },
        "..." : "..."  // All other properties
    },
    "current_depth_profile" : {
        "resolution" : { "width" : 640, "height" : 360 },
        "framerate" : [15]
    },
    "current_profile" : {
        "resolution" : { "width" : 640, "height" : 360 },
        "framerate" : [15]
    },
    "available_profiles" : {
        {
            "resolution" : { "width" : 640, "height" : 360 },
            "framerate" : [15, 30, 60]
        },
        {
            "..." : "..."
        }
    },
    "available_depth_profiles" : {
        {
            "resolution" : { "width" : 640, "height" : 360 },
            "framerate" : [15, 30, 60]
        },
        {
            "..." : "..."
        }
    },
    "color_intrinsic" : [],
    "color_distortion" : [],
    "depth_intrinsic" : [],
    "depth_distortion" : [],
    "extrinsic" : [],
}
```

All the properties are inside the field "property" as an object of type crf::expected<int> with the value of the property. If the camera does not have that property or can't read it, an error code is shown instead.

The available profiles show all the available profiles for the camera and the current profile shows the one currently selected. **Note that it might not match the one currently selected by the client for their streamer** This can happen if other clients are connected or if custom framerates are used.

The same can happen with the depth profile.

The matrixes of color, depth, and extrinsic are 3x3 represented in and array of 1x9 where the first three values represent one row, the next three the next one, and the last three the last one.
