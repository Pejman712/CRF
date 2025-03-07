@ingroup group_camera_communication_point

This is the documentation for the API of the CamerasCommunciationPoint. It includes a description of the messages to send and the expected replies.

- [Cameras Communication Point {#cameras\_communication\_point}](#cameras-communication-point-cameras_communication_point)
  - [Notes](#notes)
  - [Available API calls](#available-api-calls)
    - [Start Frame Stream](#start-frame-stream)
    - [Frame Stream](#frame-stream)
    - [Stop Frame Stream](#stop-frame-stream)
    - [Set Property](#set-property)
    - [Set Profile](#set-profile)
    - [Get Status](#get-status)

### Notes

This communication point can take a maximum of five frame streamers, with a maximum of one frame stream per client.

This communication point inherits from the [PriorityAccessCommunicationPoint]{reference_missing} class and its common functionalities.

### Available API calls

#### Start Frame Stream

This method starts a frame streamer specific to the client that requested it. The message needs to follow the next structure.

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
    "encoding_format" : 1
}
```

The parameter encoding format is an enum with the following values

- 1 JPEG encoding
- 2 cvMat encoding
- 3 x264 encoding

The parameter encoding quality is an enum that allows the user to select the compression rate. The enum is described as follows:

- 0 Ultrafast
- 1 Superfast
- 2 Veryfast
- 3 Faster
- 4 Fast
- 5 Normal
- 6 Slow
- 7 Slower
- 8 VerySlow

For more information on each encoding type and quality refer to [VideoCodecs]{reference_missing}

As for the profile, the resolution has to be a resolution available for the camera, custom resolutions are not supported. The framerate will be set on the camera and will influence the frequency at which the frame streamer runs.

When a custom framerate is selected the camera will have the maximum framerate possible and the streamer will send the data following the selected frequency.

When several streamers are requested, the camera will have the highest resolution and framerate between the selected ones.

In case of errors, the response will follow a JSONError type with a string explaining the error.
If the request was successful a JSONReply with a boolean will be received and if it's true the streamer will start.

#### Frame Stream

Once started, the frame streamer is responsible to send the frames to the client respecting the framerate requested.

The frame will be sent using a [FramePacket]{reference_missing} and the selected encoding.

#### Stop Frame Stream

This command will stop the streamer if started. It is recommended to call this method before disconnection.

```json
{
    "command" : "stopFrameStream",
}
```

If the request was successful a JSONReply with a boolean will be received and if it's true the streamer will stop.

#### Set Property

To set the property a priority is requested as it changes the physical parameters of the camera.

The next snippet shows the full structure of the message, although not all the properties need to be sent. Any subset of them is accepted

```json
{
    "command" : "setProperty",
    "priority" : 1,
    "property" : {
        "brightness" : 0,
        "contrast" : 0,
        "saturation" : 0,
        "hue" : 0,
        "gain" : 0,
        "exposure" : 0,
        "focus" : 0,
        "focus_mode" : 0,
        "shutter" : 0,
        "iso" : 0,
        "zoom" : 0,
        "pan" : 0,
        "tilt" : 0,
        "roll" : 0
    }
}
```

In case of errors, the response will follow a JSONError type with a string explaining the error.
If the request was successful a JSONReply with a boolean will be received.

#### Set Profile

Setting the profile will change the framerate and resolution of the frame streamer.

```json
{
    "command" : "setProfile",
    "image_profile" : {
        "resolution" : { "width" : 640, "height" : 360 },
        "framerate" : [15]  // IMPORTANT!!! IT'S AN ARRAY OF SIZE 1
    }
}
```

In case of errors, the response will follow a JSONError type with a string explaining the error.
If the request was successful a JSONReply with a boolean will be received.

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
    }
}
```

All the properties are inside the field "property" as an object of type crf::expected<int> with the value of the property. If the camera does not have that property or can't read it, an error code is shown instead.

The available profiles show all the available profiles for the camera and the current profile shows the one currently selected. **Note that it might not match the one currently selected by the client for their streamer** This can happen if other clients are connected or if custom framerates are used.
