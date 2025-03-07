/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 * ====================================================================
*/


#pragma once

#include <vicon/DataStreamClient.h>
#include <vicon/IDataStreamClientBase.h>

namespace viconSDK = ViconDataStreamSDK::CPP;

namespace crf::communication::viconapi {

/**
 * @ingroup group_vicon_api
 * @brief The IViconApi interface is the wrapper on the VICON library which expose all the
 *        functions of VICON to the user. It is mainly used in order to mock the VICON
 *        Library for the tests.
 */
class IViconApi {
 public:
    virtual ~IViconApi() = default;
    /**
     * @brief Discover whether client is connected to the Vicon DataStream Server.
     * @return An Output_IsConnected class containing a true value for Connected if you are
     *         connected to the stream, otherwise false.
     */
    virtual viconSDK::Output_IsConnected isConnected() const = 0;
    /**
     * @brief Disconnect from the Vicon DataStream Server.
     * @return An Output_Disconnect class containing the result of the operation.
     *         - The Result will be:
     *           + Success
     *           + NotConnected
     */
    virtual viconSDK::Output_Disconnect disconnect() = 0;
    /**
     * @brief Set connection timeout for Connect()
     *        Connect will return ClientConnectionFailed if no connection was able to be made
     *        within Timeout milliseconds, default is 5000 milliseconds, minimum is 10
     *        milliseconds.
     * @param Timeout Connection timeout in millisecond, default is 5000 minimum.
     * @return An Output_Connect class containing the result of the connect operation.
     *         - The Result will be:
     *           + Success
     *           + ArgumentOutOfRange
     */
    virtual viconSDK::Output_SetConnectionTimeout setConnectionTimeout(unsigned int Timeout) = 0;
    /**
     * @brief Establish a dedicated connection to a Vicon DataStream Server. 
     *        The function defaults to connecting on port 801. 
     *        You can specify an alternate port number after a colon.
     *        This is for future compatibility: current products serve data on port 801 only.
     *        If no connection could be made, it will return within timeout.
     * @param HostName The DNS-identifiable name, or IP address of the PC hosting
     *                 the DataStream server. For example:
     *                 + "localhost"
     *                 + "MyViconPC:801"
     *                 + "10.0.0.2"
     * @return An Output_Connect class containing the result of the connect operation.
     *         - The Result will be:
     *           + Success
     *           + InvalidHostName
     *           + ClientAlreadyConnected
     *           + ClientConnectionFailed
     */
    virtual viconSDK::Output_Connect connect(const viconSDK::String& HostName) = 0;
    /**
     * @brief Request a new frame to be fetched from the Vicon DataStream Server.
     * @return An Output_GetFrame class containing the result of the operation.
     *         - The Result will be:
     *           + Success
     *           + NotConnected
     */
    virtual viconSDK::Output_GetFrame getFrame() = 0;
    /**
     * @brief Enable kinematic segment data in the Vicon DataStream.
     *        Call this function on startup, after connecting to the server, and before
     *        trying to read local or global segment data.
     * @return An Output_EnableSegmentData class containing the result of the operation.
     *        - The Result will be:
     *          + Success
     *          + NotConnected
     */
    virtual viconSDK::Output_EnableSegmentData enableSegmentData() = 0;
    /**
     * @brief Enable labeled reconstructed marker data in the Vicon DataStream.
     *        Call this function on startup, after connecting to the server,
     *        and before trying to read labeled marker data.
     * @return An Output_EnableMarkerData class containing the result of the operation.
     *         - The Result will be:
     *           + Success
     *           + NotConnected
     */
    virtual viconSDK::Output_EnableMarkerData enableMarkerData() = 0;
    /**
     * @brief Enable unlabeled reconstructed marker data in the Vicon DataStream.
     *        Call this function on startup, after connecting to the server, and before
     *        trying to read global unlabeled marker data.
     * @return An Output_UnlabeledMarkerData class containing the result of the operation.
     *         - The Result will be:
     *           + Success
     *           + NotConnected
     */
    virtual viconSDK::Output_EnableUnlabeledMarkerData enableUnlabeledMarkerData() = 0;
    /**
     * @brief Enable a lightweight transmission protocol for kinematic segment data in the
     *        Vicon DataStream. This will reduce the network bandwidth required to transmit
     *        segment data to approximately a quarter of that required by the previous method,
     *        at the expense of a small amount of precision.
     *        Use the existing methods such as GetSegmentGlobalTranslation() and
     *        GetSegmentGlobalRotationMatrix() as usual to obtain the segment data.
     *        Calling this method will automatically disable all other configurable output types.
     *        These may be re-enabled after the call if required.
     *        Call this function on startup, after connecting to the server, and before trying to
     *        read local or global segment data.
     * @return An Output_EnableSegmentData class containing the result of the operation.
     *         - The Result will be:
     *           + Success
     *           + NotConnected
     */
    virtual viconSDK::Output_EnableLightweightSegmentData enableLightweightSegmentData() = 0;
    /**
     * @brief There are three modes that the SDK can operate in. Each mode has a different
     *        impact on the Client, Server, and network resources used.
     *        + **ServerPush**
     *        In "ServerPush" mode, the Server pushes every new frame of data over the network
     *        to the Client. The Server will try not to drop any frames.
     *        This results in the lowest latency that can be achieved.
     *        If the Client is unable to read data at the rate it is being sent, then it is
     *        buffered, firstly in the Client, then on the TCP/IP connection, and then at the
     *        Server. When all the buffers are full then frames may be dropped at the Server and
     *        the performance of the Server may be affected. The GetFrame() method returns the most
     *        recently received frame if available, or blocks the calling thread if the most
     *        recently received frame has already been processed.
     *
     *        + **ClientPull**
     *        In "ClientPull" mode, the Client waits for a call to GetFrame(), and then requests
     *        the latest frame of data from the Server. This increases latency, because a request
     *        must be sent over the network to the Server, the Server has to prepare the frame
     *        of data for the Client, and then the data must be sent back over the network.
     *        Network bandwidth is kept to a minimum, because the Server only sends what you need.
     *        The buffers are very unlikely to be filled, and Server performance is unlikely to be
     *        affected. The GetFrame() method blocks the calling thread until the frame has been
     *        received.
     *
     *        + **ClientPullPreFetch**
     *        "ClientPullPreFetch" is an enhancement to the "ClientPull" mode.
     *        As soon as a sample has been received, it will preemptively request the next sample.
     *        The server will send you this next sample as soon as it is ready, so do not
     *        experience the delay in requesting it. GetFrame() may block the calling thread if
     *        the next frame has not been received yet.
     *        As with normal "ClientPull", buffers are unlikely to fill up, and Server performance
     *        is unlikely to be affected.
     *
     *        The stream defaults to "ClientPull" mode as this is considered the easiest option.
     *        For improved performance use "ServerPush". "ClientPullPreFetch" may be useful if
     *        problems are being caused by large numbers of samples being buffered.
     * @param Mode Stream modes that the SDK can operate in
     *             + StreamMode.ServerPush
     *             + StreamMode.ClientPull
     *             + StreamMode.ClientPullPreFetch
     * @return An Output_SetStreamMode class containing the result of the operation.
     *         - The Result will be:
     *           + Success
     *           + NotConnected
     */
    virtual viconSDK::Output_SetStreamMode setStreamMode(
        const viconSDK::StreamMode::Enum Mode) = 0;
    /**
     * @brief Remaps the 3D axis.
     *         Vicon Data uses a right-handed coordinate system, with +X forward, +Y left, and
     *         +Z up. Other systems use different coordinate systems. The SDK can transform 
     *         its data into any valid right-handed coordinate system by re-mapping each axis.
     *         Valid directions are "Up", "Down", "Left", "Right", "Forward", and "Backward".
     *         Note that "Forward" means moving away from you, and "Backward" is moving 
     *         towards you.
     *         Common usages are
     *         Z-up: SetAxisMapping( Forward, Left, Up )
     *         Y-up: SetAxisMapping( Forward, Up, Right )
     * @param XAxis Specify the direction of your X axis relative to yourself as the observer.
     * @param YAxis Specify the direction of your Y axis relative to yourself as the observer.
     * @param ZAxis Specify the direction of your Z axis relative to yourself as the observer.
     * @return An Output_SetAxisMapping class containing the result of the operation.
     *         - The Result will be:
     *           + Success
     *           + CoLinearAxes
     *           + LeftHandedAxes
     */
    virtual viconSDK::Output_SetAxisMapping setAxisMapping(const viconSDK::Direction::Enum XAxis,
        const viconSDK::Direction::Enum YAxis,
        const viconSDK::Direction::Enum ZAxis) = 0;
    /**
     * @brief Set the number of frames that the client should buffer.
     *        The default value is 1, which always supplies the latest frame.
     *        Choose higher values to reduce the risk of missing frames between calls.
     * @param BufferSize The maximum number of frames to buffer.
     * @return Nothing
     */
    virtual void setBufferSize(unsigned int BufferSize) = 0;
    /**
     * @brief Output timing information to a log file
     * @param ClientLog
     * @param StreamLog
     * @return 
     */
    virtual viconSDK::Output_SetTimingLogFile setTimingLogFile(const viconSDK::String& ClientLog,
        const viconSDK::String& StreamLog) = 0;
    /**
     * @brief Return the number of subjects in the DataStream. This information can be used in
     *        conjunction with GetSubjectName.
     * @return An Output_GetSubjectCount class containing the result of the operation and the
     *         number of subjects.
     *         - The Result will be:
     *           + Success
     *           + NotConnected
     *           + NoFrame
     */
    virtual viconSDK::Output_GetSubjectCount getSubjectCount() const = 0;
    /**
     * @brief Return the name of a subject. This can be passed into segment and marker functions.
     * @param SubjectIndex The index of the subject. A valid Subject Index is between 0 and
     *                     GetSubjectCount()-1.
     * @return An Output_GetSubjectName GetSubjectName class containing the result of the operation
     *         and the name of the subject.
     *         - The Result will be:
     *           + Success
     *           + NotConnected
     *           + NoFrame
     *           + InvalidIndex
     */
    virtual viconSDK::Output_GetSubjectName getSubjectName(
        const unsigned int SubjectIndex) const = 0;
    /**
     * @brief Return the translation of a subject segment in global coordinates.
     *        The translation is of the form (x, y, z) where x, y and z are in millimeters with
     *        respect to the global origin.
     * @param SubjectName The name of the subject.
     * @param SegmentName The name of the segment.
     * @return An Output_GetSegmentGlobalTranslation class containing the result of the operation,
     *         the translation of the segment, and whether the segment is occluded.
     *         - The Result will be:
     *            + Success
     *            + NotConnected
     *            + NoFrame
     *            + InvalidSubjectName
     *            + InvalidSegmentName
     *         - Occluded will be True if the segment was absent at this frame. In this case
     *           the translation will be [0,0,0].
     */
    virtual viconSDK::Output_GetSegmentGlobalTranslation getSegmentGlobalTranslation(
        const viconSDK::String& SubjectName, const viconSDK::String& SegmentName) const = 0;
    /**
     * @brief Return the rotation of a subject segment in global quaternion coordinates.
     *        The quaternion is of the form (x, y, z, w) where w is the real component and
     *        x, y and z are the imaginary components.
     *        N.B. This is different from that used in many other applications, which use
     *        (w, x, y, z).
     * @param SubjectName The name of the subject.
     * @param SegmentName The name of the segment.
     * @return An Output_GetSegmentGlobalRotationQuaternion  class containing the result of the
     *         operation, the rotation of the segment, and whether the segment is occluded.
     *          - The Result will be:
     *            + Success
     *            + NotConnected
     *            + NoFrame
     *            + InvalidSubjectName
     *            + InvalidSegmentName
     *          - Occluded will be True if the segment was absent at this frame. In this case the
     *            Rotation will be [1,0,0,0].
     */
    virtual viconSDK::Output_GetSegmentGlobalRotationQuaternion getSegmentGlobalRotationQuaternion(
        const viconSDK::String& SubjectName, const viconSDK::String& SegmentName) const = 0;
    /**
     * @brief Return the number of markers for a specified subject in the DataStream. This
     *        information can be used in conjunction with GetMarkerName.
     * @param SubjectName The name of the subject.
     * @return An Output_GetMarkerCount class containing the result of the operation, and the
     *         number of markers.
     *         - The Result will be:
     *           + Success
     *           + NotConnected
     *           + NoFrame
     *           + InvalidSubjectName
     */
    virtual viconSDK::Output_GetMarkerCount getMarkerCount(
        const viconSDK::String& SubjectName) const = 0;
    /**
     * @brief Return the name of a marker for a specified subject. This can be passed into
     *        GetMarkerGlobalTranslation.
     * @param SubjectName The name of the subject.
     * @param MarkerIndex The index of the marker.
     * @return An Output_GetMarkerName class containing the result of the operation and the name
     *         of the marker.
     *         - The Result will be:
     *           + Success
     *           + NotConnected
     *           + NoFrame
     *           + InvalidSubjectName
     *           + InvalidIndex
     */
    virtual viconSDK::Output_GetMarkerName getMarkerName(const viconSDK::String& SubjectName,
        const unsigned int MarkerIndex) const = 0;
    /**
     * @brief Return the translation of a subject marker in global coordinates.
     *        The Translation is of the form ( x, y, z ) where x, y and z are in millimeters with
     *        respect to the global origin.
     * @param SubjectName The name of the subject.
     * @param MarkerName The name of the marker.
     * @return An Output_GetMarkerGlobalTranslation class containing the result of the operation,
     *         the translation of the marker, and whether the marker is occluded.
     *         - The Result will be:
     *           + Success
     *           + NotConnected
     *           + NoFrame
     *           + InvalidSubjectName
     *           + InvalidMarkerName
     *         - Occluded will be true if the marker was absent at this frame. In this case the
     *           Translation will be [0,0,0].
     */
    virtual viconSDK::Output_GetMarkerGlobalTranslation getMarkerGlobalTranslation(
        const viconSDK::String& SubjectName, const viconSDK::String& MarkerName) const = 0;
};

}  // namespace crf::communication::viconapi
