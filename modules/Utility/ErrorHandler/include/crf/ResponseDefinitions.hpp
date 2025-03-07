/*
 * © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *         Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/
#pragma once

namespace crf {

/**
 * @ingroup group_error_handler
 * @brief The code of a response is a FOUR-digit integer code that describes the result of the
 *        request and the semantics of the response, including whether the request was successful
 *        and what content is enclosed (if any). This list is based on the Hypertext Transfer
 *        Protocol (HTTP) status codes desribed in [RFC9110], but it is expanded to allow more
 *        detailed information of the framework. All valid status codes are within the range of 100
 *        to 4999, inclusive.
 *
 *        The first two digits of the status code define the class of response. The last two digits
 *        do not have any categorization role:
 *          - 0xxx (General)
 *            - 00xx (Not used): Not usable
 *            - 01xx (Informational): The request was received, continuing process.
 *            - 02xx (Successful): The request was successfully received, understood, and accepted.
 *            - 03xx (Redirection): Further action needs to be taken to complete the request.
 *            - 04xx (Client Error): The request contains bad syntax or cannot be fulfilled.
 *            - 05xx (Server Error): The server failed to fulfill an apparently valid request.
 *            - 06xx (General Hardware Status):
 *            - 07xx (Undefined):
 *            - 08xx (Control Status):
 *            - 09xx (Undefined):
 *          - 1xxx (Communications Protocols (CANopen, EtherCAT, ...))
 *            - 10xx (CANopen): Specific error codes realted to the CANopen protocol
 *            - 11xx (EtherCAT): Specific error codes realted to the EtherCAT protocol
 *            - 12xx (Undefined):
 *            - 13xx (Undefined):
 *            - 14xx (Undefined):
 *            - 15xx (Undefined):
 *            - 16xx (Undefined):
 *            - 17xx (Undefined):
 *            - 18xx (Undefined):
 *            - 19xx (Undefined):
 *          - 2xxx (Proprietary Robots)
 *            - 21xx (Universal Robot): Specific information regarding the UR robots status.
 *            - 22xx (Haption): Specific information regarding the Haption devices status.
 *            - 23xx (KinovaJaco2): Specific error codes for the Kinova Jaco 2 Robotic Arm
 *            - 24xx (Undefined):
 *            - 25xx (Undefined):
 *            - 26xx (Undefined):
 *            - 27xx (Undefined):
 *            - 28xx (Undefined):
 *            - 29xx (Undefined):
 *          - 3xxx (Proprietary Sensors)
 *            - 31xx (Vicon Motion Capture): Specific error codes for Vicon Motion Capture
 *            - 32xx (Undefined):
 *            - 33xx (Undefined):
 *            - 34xx (Undefined):
 *            - 35xx (Undefined):
 *            - 36xx (Undefined):
 *            - 37xx (Undefined):
 *            - 38xx (Undefined):
 *            - 39xx (Undefined):
 *          - 4xxx (Undefined)
 *            - 41xx (Undefined):
 *            - 42xx (Undefined):
 *            - 43xx (Undefined):
 *            - 44xx (Undefined):
 *            - 45xx (Undefined):
 *            - 46xx (Undefined):
 *            - 47xx (Undefined):
 *            - 48xx (Undefined):
 *            - 49xx (Undefined):
 *
 *        Refrences
 *         - RFC9110: https://datatracker.ietf.org/doc/html/rfc9110
 *
 */
enum class Code {
    /**
     * @brief Unknown reponse code
     */
    Empty = 0,
    /**
     * @brief Indicates that the initial part of a request has been received and has not yet been
     *        rejected by the server.  The server intends to send a final response after the
     *        request has been fully received and acted upon. Check the section 15.2.1 of RFC9110
     *        for more information.
     */
    Continue = 100,
    /**
     * @brief Indicates that the server understands and is willing to comply with the client's
     *        request, via the Upgrade header field, for a change in the application protocol being
     *        used on this connection. The server must generate an Upgrade header field in the
     *        response that indicates which protocol(s) will be in effect after this response.
     *        Check the section 15.2.2 of RFC9110 for more information.
     */
    SwitchingProtocols = 101,
    /**
     * @brief Indicates that the request has succeeded. The content sent in a 200 response depends
     *        on the request method. Check the section 15.3.1 of RFC9110 for more information.
     */
    OK = 200,
    /**
     * @brief Indicates that the request has been fulfilled and has resulted in one or more new
     *        resources being created.  The primary resource created by the request is identified
     *        by either a Location header field in the response or, if no Location header field is
     *        received, by the target URI. Check the section 15.3.2 of RFC9110 for more
     *        information.
     */
    Created = 201,
    /**
     * @brief Indicates that the request has been accepted for processing, but the processing has
     *        not been completed. The request might or might not eventually be acted upon, as it
     *        might be disallowed when processing actually takes place. There is no facility in
     *        HTTP for re-sending a status code from an asynchronous operation. Check the section
     *        p15.3.3 of RFC9110 for more information.
     */
    Accepted = 202,
    /**
     * @brief Indicates that the request was successful but the enclosed content has been modified
     *        from that of the origin server's 200 (OK) response by a transforming proxy. This
     *        status code allows the proxy to notify recipients when a transformation has been
     *        applied, since that knowledge might impact later decisions regarding the content. For
     *        example, future cache validation requests for the content might only be applicable
     *        along the same request path (through the same proxies). Check the section 15.3.4 of
     *        RFC9110 for more information.
     */
    NonAuthoritativeInformation = 203,
    /**
     * @brief Indicates that the server has successfully fulfilled the request and that there is no
     *        additional content to send in the response content. Metadata in the response header
     *        fields refer to the target resource and its selected representation after the
     *        requested action was applied. Check the section 15.3.5 of RFC9110 for more
     *        information.
     */
    NoContent = 204,
    /**
     * @brief Indicates that the server has fulfilled the request and desires that the user agent
     *        reset the "document view", which caused the request to be sent, to its original state
     *        as received from the origin server. Check the section 15.3.6 of RFC9110 for more
     *        information.
     */
    ResetContent = 205,
    /**
     * @brief Indicates that the server is successfully fulfilling a range request for the target
     *        resource by transferring one or more parts of the selected representation. Check the
     *        section 15.3.7 of RFC9110 for more information.
     */
    PartialContent = 206,
    /**
     * @brief Indicates that the target resource has more than one representation, each with its
     *        own more specific identifier, and information about the alternatives is being
     *        provided so that the user (or user agent) can select a preferred representation by
     *        redirecting its request to one or more of those identifiers. In other words, the
     *        server desires that the user agent engage in reactive negotiation to select the most
     *        appropriate representation(s) for its needs. Check the section 15.4.1 of RFC9110 for
     *        more information.
     */
    MultipleChoices = 300,
    /**
     * @brief Indicates that the target resource has been assigned a new permanent URI and any
     *        future references to this resource ought to use one of the enclosed URIs. The
     *        server is suggesting that a user agent with link-editing capability can permanently
     *        replace references to the target URI with one of the new references sent by the
     *        server. However, this suggestion is usually ignored unless the user agent is actively
     *        editing references (e.g., engaged in authoring content), the connection is secured,
     *        and the origin server is a trusted authority for the content being edited. Check the
     *        section 15.4.2 of RFC9110 for more information.
     */
    MovedPermanently = 301,
    /**
     * @brief Indicates that the target resource resides temporarily under a different URI. Since
     *        the redirection might be altered on occasion, the client ought to continue to use the
     *        target URI for future requests.Check the section 15.4.3 of RFC9110 for more
     *        information.
     */
    Found = 302,
    /**
     * @brief Indicates that the server is redirecting the user agent to a different resource, as
     *        indicated by a URI in the Location header field, which is intended to provide an
     *        indirect response to the original request. A user agent can perform a retrieval
     *        request targeting that URI (a GET or HEAD request if using HTTP), which might also be
     *        redirected, and present the eventual result as an answer to the original request.
     *        Note that the new URI in the Location header field is not considered equivalent to
     *        the target URI. Check the section 15.4.4 of RFC9110 for more information.
     */
    SeeOther = 303,
    /**
     * @brief Indicates that a conditional GET or HEAD request has been received and would have
     *        resulted in a 200 (OK) response if it were not for the fact that the condition
     *        evaluated to false.  In other words, there is no need for the server to transfer a
     *        representation of the target resource because the request indicates that the client,
     *        which made the request conditional, already has a valid representation; the server is
     *        therefore redirecting the client to make use of that stored representation as if it
     *        were the content of a 200 (OK) response. Check the section 15.4.5 of RFC9110 for more
     *        information.
     */
    NotModified = 304,
    /**
     * @brief This status code was defined in a previous version of this specification and is now
     *        deprecated. Check the appendix B of RFC7231 for more information.
     */
    UseProxy = 305,
    /**
     * @brief This status code was defined in a previous version of the specification RFC9110, is
     *        no longer used, and the code is reserved.
     */
    Unused1 = 306,
    /**
     * @brief Indicates that the target resource resides temporarily under a different URI and the
     *        user agent must NOT change the request method if it performs an automatic redirection
     *        to that URI. Since the redirection can change over time, the client ought to continue
     *        using the original target URI for future requests. Check the section 15.4.8 of
     *        RFC9110 for more information.
     */
    TemporaryRedirect = 307,
    /**
     * @brief Indicates that the target resource has been assigned a new permanent URI and any
     *        future references to this resource ought to use one of the enclosed URIs. The server
     *        is suggesting that a user agent with link-editing capability can permanently replace
     *        references to the target URI with one of the new references sent by the server.
     *        However, this suggestion is usually ignored unless the user agent is actively editing
     *        references (e.g., engaged in authoring content), the connection is secured, and the
     *        origin server is a trusted authority for the content being edited. Check the section
     *        15.4.9 of RFC9110 for more information.
     */
    PermanentRedirect = 308,
    /**
     * @brief Indicates that the server cannot or will not process the request due to something
     *        that is perceived to be a client error (e.g., malformed request syntax, invalid
     *        request message framing, or deceptive request routing). Check the section 15.5.1 of
     *        RFC9110 for more information.
     */
    BadRequest = 400,
    /**
     * @brief Indicates that the request has not been applied because it lacks valid authentication
     *        credentials for the target resource.  The server generating a 401 response must send
     *        a WWW-Authenticate header field containing at least one challenge applicable to the
     *        target resource. Check the section 15.5.2 of RFC9110 for more information.
     */
    Unauthorized = 401,
    /**
     * @brief This status code is reserved for future use.
     */
    PaymentRequired = 402,
    /**
     * @brief Indicates that the server understood the request but refuses to fulfill it. A server
     *        that wishes to make public why the request has been forbidden can describe that
     *        reason in the response content (if any). Check the section 15.5.4 of RFC9110 for more
     *        information.
     */
    Forbidden = 403,
    /**
     * @brief Indicates that the origin server did not find a current representation for the target
     *        resource or is not willing to disclose that one exists. A 404 status code does not
     *        indicate whether this lack of representation is temporary or permanent; the 410
     *        (Gone) status code is preferred over 404 if the origin server knows, presumably
     *        through some configurable means, that the condition is likely to be permanent. Check
     *        the section 15.5.5 of RFC9110 for more information.
     */
    NotFound = 404,
    /**
     * @brief Indicates that the method received in the request-line is known by the origin server
     *        but not supported by the target resource. The origin server must generate an Allow
     *        header field in a 405 response containing a list of the target resource's currently
     *        supported methods. Check the section 15.5.6 of RFC9110 for more information.
     */
    MethodNotAllowed = 405,
    /**
     * @brief Indicates that the target resource does not have a current representation that would
     *        be acceptable to the user agent, according to the proactive negotiation header fields
     *        received in the request, and the server is unwilling to supply a default
     *        representation. Check the section 15.5.7 of RFC9110 for more information.
     */
    NotAcceptable = 406,
    /**
     * @brief It is similar to 401 (Unauthorized), but it indicates that the client needs to
     *        authenticate itself in order to use a proxy for this request. The proxy MUST send a
     *        Proxy-Authenticate header field containing a challenge applicable to that proxy for
     *        the request. The client MAY repeat the request with a new or replaced
     *        Proxy-Authorization header field. Check the section 15.5.8 of RFC9110 for more
     *        information.
     */
    ProxyAuthenticationRequired = 407,
    /**
     * @brief Indicates that the server did not receive a complete request message within the time
     *        that it was prepared to wait. Check the section 15.5.9 of RFC9110 for more
     *        information.
     */
    RequestTimeout = 408,
    /**
     * @brief Indicates that the request could not be completed due to a conflict with the current
     *        state of the target resource. This code is used in situations where the user might be
     *        able to resolve the conflict and resubmit the request. The server should generate
     *        content that includes enough information for a user to recognize the source of the
     *        conflict. Check the section 15.5.1 of RFC9110 for more information.
     */
    Conflict = 409,
    /**
     * @brief Indicates that access to the target resource is no longer available at the origin
     *        server and that this condition is likely to be permanent. If the origin server does
     *        not know, or has no facility to determine, whether or not the condition is permanent,
     *        the status code 404 (Not Found) ought to be used instead. Check the section 15.5.1 of
     *        RFC9110 for more information.
     */
    Gone = 410,
    /**
     * @brief Indicates that the server refuses to accept the request without a defined
     *        Content-Length. The client MAY repeat the request if it adds a valid Content-Length
     *        header field containing the length of the request content. Check the section 15.5.1
     *        of RFC9110 for more information.
     */
    LengthRequired = 411,
    /**
     * @brief Indicates that one or more conditions given in the request header fields evaluated to
     *        false when tested on the server. This response status code allows the client to place
     *        preconditions on the current resource state (its current representations and
     *        metadata) and, thus, prevent the request method from being applied if the target
     *        resource is in an unexpected state. Check the section 15.5.1 of RFC9110 for more
     *        information.
     */
    PreconditionFailed = 412,
    /**
     * @brief Indicates that the server is refusing to process a request because the request
     *        content is larger than the server is willing or able to process. The server may
     *        terminate the request, if the protocol version in use allows it; otherwise, the
     *        server may close the connection.Check the section 15.5.1 of RFC9110 for more
     *        information.
     */
    ContentTooLarge = 413,
    /**
     * @brief Indicates that the server is refusing to service the request because the target URI
     *        is longer than the server is willing to interpret. This rare condition is only likely
     *        to occur when a client has improperly converted a POST request to a GET request with
     *        long query information, when the client has descended into an infinite loop of
     *        redirection (e.g., a redirected URI prefix that points to a suffix of itself) or when
     *        the server is under attack by a client attempting to exploit potential security holes.
     *        Check the section 15.5.1 of RFC9110 for more information.
     */
    URITooLong = 414,
    /**
     * @brief Indicates that the origin server is refusing to service the request because the
     *        content is in a format not supported by this method on the target resource. Check the
     *        section 15.5.1 of RFC9110 for more information.
     */
    UnsupportedMediaType = 415,
    /**
     * @brief Indicates that the set of ranges in the request's Range header field has been rejected
     *        either because none of the requested ranges are satisfiable or because the client has
     *        requested an excessive number of small or overlapping ranges (a potential denial of
     *        service attack). Check the section 15.5.1 of RFC9110 for more information.
     */
    RangeNotSatisfiable = 416,
    /**
     * @brief Indicates that the expectation given in the request's Expect header field could not
     *        be met by at least one of the inbound servers. Check the section 15.5.1 of RFC9110
     *        for more information.
     */
    ExpectationFailed = 417,
    /**
     * @brief The 418 status code is reserved in the IANA HTTP Status Code Registry. This indicates
     *        that the status code cannot be assigned to other applications currently. Check the
     *        section 15.5.1 of RFC9110 for more information.
     */
    Unused2 = 418,
    /**
     * @brief Indicates that the request was directed at a server that is unable or unwilling to
     *        produce an authoritative response for the target URI. An origin server (or gateway
     *        acting on behalf of the origin server) sends 421 to reject a target URI that does not
     *        match an origin for which the server has been configured or does not match the
     *        connection context over which the request was received. Check the section 15.5.2 of
     *        RFC9110 for more information.
     */
    MisdirectedRequest = 421,
    /**
     * @brief Indicates that the server understands the content type of the request content (hence
     *        a 415 (Unsupported Media Type) status code is inappropriate), and the syntax of the
     *        request content is correct, but it was unable to process the contained instructions.
     *        For example, this status code can be sent if an XML request content contains
     *        well-formed (i.e., syntactically correct), but semantically erroneous XML
     *        instructions. Check the section 15.5.2 of RFC9110 for more information.
     */
    UnprocessableContent = 422,
    /**
     * @brief Indicates that the server refuses to perform the request using the current protocol
     *        but might be willing to do so after the client upgrades to a different protocol. The
     *        server MUST send an Upgrade header field in a 426 response to indicate the required
     *        protocol(s). Check the section 15.5.2 of RFC9110 for more information.
     */
    UpgradeRequired = 426,
    /**
     * @brief Indicates that the server encountered an unexpected condition that prevented it from
     *        fulfilling the request. Check the section 15.6.1 of RFC9110 for more information.
     */
    InternalServerError = 500,
    /**
     * @brief Indicates that the server does not support the functionality required to fulfill the
     *        request. This is the appropriate response when the server does not recognize the
     *        request method and is not capable of supporting it for any resource. Check the
     *        section 15.6.2 of RFC9110 for more information.
     */
    NotImplemented = 501,
    /**
     * @brief Indicates that the server, while acting as a gateway or proxy, received an invalid
     *        response from an inbound server it accessed while attempting to fulfill the request.
     *        Check the section 15.6.3 of RFC9110 for more information.
     */
    BadGateway = 502,
    /**
     * @brief Indicates that the server is currently unable to handle the request due to a
     *        temporary overload or scheduled maintenance, which will likely be alleviated after
     *        some delay. The server may send a Retry-After header field to suggest an appropriate
     *        amount of time for the client to wait before retrying the request. Check the section
     *        15.6.4 of RFC9110 for more information.
     */
    ServiceUnavailable = 503,
    /**
     * @brief Indicates that the server, while acting as a gateway or proxy, did not receive a
     *        timely response from an upstream server it needed to access in order to complete the
     *        request. Check the section 15.6.5 of RFC9110 for more information.
     */
    GatewayTimeout = 504,
    /**
     * @brief Indicates that the server does not support, or refuses to support, the major version
     *        of HTTP that was used in the request message. The server is indicating that it is
     *        unable or unwilling to complete the request using the same major version as the
     *        client, other than with this error message. The server should generate a
     *        representation for the 505 response that describes why that version is not supported
     *        and what other protocols are supported by that server. Check the section 15.6.6 of
     *        RFC9110 for more information.
     */
    HTTPVersionNotSupported = 505,
    /**
     * @brief Indicates that the server was not able to proccess the request because it is trying
     *        to intialize a device or service that it is already initialized.
     */
    AlreadyInitialized = 506,
    /**
     * @brief Indicates that the server was not able to proccess the request becasue the device or
     *        service is no initialized.
     */
    NotInitialized = 507,
    /**
     * @brief Indicates that a query was performed to a third party and it failed. Functions that return
     *        errors or exceptions thrown by 3rd parties are included.
     */
    ThirdPartyQueryFailed = 508,
    /**
     * @brief Indicates an error related to memory allocation in the server side
     */
    MemoryAllocationError = 509,
    /**
     * @brief 
     */
    UnstableConnection = 510,
    /**
     * @brief The device is turned on.
     */
    PoweredOn = 600,
    /**
     * @brief The device is turned off.
     */
    PoweredOff = 601,
    /**
     * @brief The device is ready to be used.
     */
    Idle = 602,
    /**
     * @brief There is no connection with the device.
     */
    Disconnected = 603,
    /**
     * @brief The emergency stop is trigered in the device.
     */
    EmergencyStop = 604,
    /**
     * @brief The device is in fault state.
     */
    FaultState = 605,
    /**
     * @brief The request to the device has failed. General error in case specification is not
     *        possible.
     */
    RequestToDeviceFailed = 606,
    /**
     * @brief The software watchdog on the connection was triggered.
     */
    ConnectionWatchdog = 606,
    /**
     * @brief The temperature in the device is not within admissible values.
     */
    TemperatureError = 607,
    /**
     * @brief A physical limit has been reached.
     */
    HardwareLimit = 608,
    /**
     * @brief The deadmean switch is released.
     */
    DeadmanSwitchReleased = 609,
    /**
     * @brief The device is calibrated.
     */
    Calibrated = 610,
    /**
     * @brief The device need calibration.
     */
    NotCalibrated = 611,
    /**
     * @brief Control error message indicating that the real time requirements have not been
     *        fulfilled.
     */
    RealTimeViolation = 800,
    /**
     * @brief A control error message that states that the robot is close to a singularity
     */
    LowManipulability = 801,
    /**
     * @brief The desired end-effector position has not been achieved inside the desired
     *        position tolerance. It migth be due to workspaceViolation or other reasons.
     */
    EndEffectorToleranceViolation = 802,
    /**
     * @brief The end efector desired position is not reachable
     */
    WorkspaceViolation = 804,
    /**
     *@brief EMCY message received from the slave
    */
    CiA301EMCY = 1000,
    /**
     *@brief The slave is in fault
    */
    CiA402MotorInFault = 1001,
    /**
     *@brief The slave is in quickstop
    */
    CiA402MotorInQuickStop = 1002,
    /**
     *@brief The mode of operation is not possible
    */
    CiA402NoModesOfOperationPossible = 1003,
    /**
     *@brief Changing mode of operation gave a timeout
    */
    CiA402ModeOfOperationTimeout = 1004,
    /**
     *@brief Trying to transiton gave a timeout
    */
    CiA402TransitionTimeout = 1005,
    /**
     * @brief An SDO write request to a CANopen slave failed.
     *
     */
    SDOWriteAbort = 1006,
    /**
     * @brief An SDO read request to a CANopen slave failed.
     *
     */
    SDOReadAbort = 1007,
    /**
     * @brief A following error got triggered inside the Profile Position Mode in CANopen
     *
     */
    PPM_FollowingError = 1008,
    /**
     * @brief The target was reached inside the Profile Position Mode in CANopen
     *
     */
    PPM_TargetReached = 1009,
    /**
     * @brief The new point got set inside the Profile Position Mode in CANopen
     *
     */
    PPM_SetPointAck = 1010,
    /**
     * @brief The speed is limited inside the Velocity Mode in CANopen
     *
     */
    VOM_SpeedLimited = 1011,
    /**
     * @brief The target was reached inside the Profile Velocity Mode in CANopen
     *
     */
    PVM_TargetReached = 1012,
    /**
     * @brief The speed is zero inside the Profile Velocity Mode in CANopen
     *
     */
    PVM_SpeedZero = 1013,
    /**
     * @brief The speed is zero limited the Profile Velocity Mode in CANopen
     *
     */
    PVM_SpeedLimited = 1014,
    /**
     * @brief The maximum slippage allowed got reached inside the Profile Velocity Mode in CANopen
     *
     */
    PVM_MaxSlippage = 1015,
    /**
     * @brief The target was reached inside the Profile Torque Mode in CANopen
     *
     */
    PTM_TargetReached = 1016,
    /**
     * @brief The torque is limited inside the Profile Torque Mode in CANopen
     *
     */
    PTM_TorqueLimited = 1017,
    /**
     * @brief The target was reached inside the Interpolated position Mode in CANopen
     *
     */
    IPM_TargetReached = 1018,
    /**
     * @brief Interpolated position Mode set in CANopen
     *
     */
    IPM_ModeSet = 1019,
    /**
     * @brief Following error inside the Interpolated position Mode in CANopen
     *
     */
    IPM_FollowingError = 1020,
    /**
     * @brief The following command didn't arrive in time inside the Cyclic Synchornous Position Mode in CANopen
     *
     */
    CSP_FollowingCommand = 1021,
    /**
     * @brief Following error inside the Cyclic Synchornous Position Mode in CANopen
     *
     */
    CSP_FollowingError = 1022,
    /**
     * @brief The following command didn't arrive in time inside the Cyclic Synchornous Velocity Mode in CANopen
     *
     */
    CSV_FollowingCommand = 1023,
    /**
     * @brief The following command didn't arrive in time inside the Cyclic Synchornous Torque Mode in CANopen
     *
     */
    CST_FollowingCommand = 1024,
    /**
    * @brief A error message that states if the EtherCat motor is not enabled.
     */
    ECMotorNotEnabled = 1100,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot mode value.
     */
    UR_UpdatingFirmware = 2100,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot status bits.
     */
    UR_TeachButtonPressed = 2101,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot status bits.
     */
    UR_PowerButtonPressed = 2102,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot mode value.
     */
    UR_NoController = 2103,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot mode value.
     */
    UR_BackDrive = 2104,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot mode value.
     */
    UR_Booting = 2105,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot mode value.
     */
    UR_Running = 2106,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot status bits.
     */
    UR_ProgramRunning = 2107,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot safety status bits.
     */
    UR_NormalMode = 2108,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot safety status bits.
     */
    UR_ReducedMode = 2109,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot safety status bits.
     */
    UR_RecoveryMode = 2110,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot safety status bits.
     */
    UR_ProtectiveStop = 2111,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot safety status bits.
     */
    UR_SafeguardStop = 2112,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot safety status bits.
     */
    UR_SystemEmergencyStop = 2113,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot safety status bits.
     */
    UR_RobotEmergencyStop = 2114,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot safety status bits.
     */
    UR_StoppedDueToSafety = 2115,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot mode value.
     */
    UR_ConfirmSafety = 2116,
    /**
     * @brief This code is specific of the Universal Robot RTDE protocol. It is defined by the
     *        robot safety status bits.
     */
    UR_Violation = 2117,
    /**
     * @brief The Kinova API is unable to load the communication layer
     *
     */
    KJ_ErrorLoadCommunicationDll = 2300,
    /**
     * @brief The method to initialize the communications in the kinova returned an error
     *
     */
    KJ_ErrorInitCommunicationMethod = 2301,
    /**
     * @brief The method to close the communications in the kinova returned an error
     *
     */
    KJ_ErrorCloseMethod = 2302,
    /**
     * @brief The method to get the device count in the kinova returned an error
     *
     */
    KJ_ErrorGetDeviceCountMethod = 2303,
    /**
     * @brief The method to send the packet from the kinova API returned an error
     *
     */
    KJ_ErrorSendPacketMethod = 2304,
    /**
     * @brief The method to set the active device in the kinova returned an error
     *
     */
    KJ_ErrorSetActiveDeviceMethod = 2305,
    /**
     * @brief The method to set the active device in the kinova returned an error
     *
     */
    KJ_ErrorGetDevicesListMethod = 2306,
    /**
     * @brief The method to scan for new devices in the kinova returned an error
     *
     */
    KJ_ErrorScanForNewDevice = 2307,
    /**
     * @brief The method to set the active device in the kinova returned an error
     *
     */
    KJ_ErrorGetActiveDeviceMethod = 2308,
    /**
     * @brief The method to open the RS485 port returned an error
     *
     */
    KJ_ErrorOpenRS485Activate = 2309,
    /**
     * @brief Error from the input/outpur device
     *
     */
    KJ_InputOutputError = 2310,
    /**
     * @brief An invalid parameter was provided to the kinova API
     *
     */
    KJ_InvalidParameter = 2311,
    /**
     * @brief Permission denied to the API functions
     *
     */
    KJ_AccessDenied = 2312,
    /**
     * @brief The specified device does not exist in the network
     *
     */
    KJ_NoSuchDevice = 2313,
    /**
     * @brief The specified entity was not found
     *
     */
    KJ_EntityNotFound = 2314,
    /**
     * @brief The device is currently busy with other tasks
     *
     */
    KJ_ResourceBusy = 2315,
    /**
     * @brief The requested operation timed out
     *
     */
    KJ_OperationTimedOut = 2316,
    /**
     * @brief The kinova API suffered an overflow
     *
     */
    KJ_Overflow = 2317,
    /**
     * @brief There was a Pipe error in the Kinova API
     *
     */
    KJ_PipeError = 2318,
    /**
     * @brief A call to the system through the Kinova API was interrupted
     *
     */
    KJ_SystemCallInterrupted = 2319,
    /**
     * @brief The device does not have enough memory to perform the requested operation
     *
     */
    KJ_InsufficientMemory = 2320,
    /**
     * @brief The requested operation is not supported in the Kinova API
     *
     */
    KJ_OperationNotSupported = 2321,
    /**
     * @brief A non-registered error was triggered in the Kinova API
     *
     */
    KJ_OtherError = 2322,
    /**
     * @brief The specified device was not found in the network
     *
     */
    KJ_ErrorNoDeviceFound = 2323,
    /**
     * @brief The Kinova API could not be initialized
     *
     */
    KJ_ErrorAPINotInitialize = 2324,
    /**
     * @brief The result is unknown. Treat it as a failure.
     *
     */
    Vicon_Unknown = 3100,
    /**
     * @brief The function called has not been implemented in this version of the SDK.
     *
     */
    Vicon_NotImplemented = 3101,
    /**
     * @brief The function call succeeded.
     *
     */
    Vicon_Success = 3102,
    /**
     * @brief The "HostName" parameter passed to Connect() is invalid.
     *
     */
    Vicon_InvalidHostName = 3103,
    /**
     * @brief The "MulticastIP" parameter was not in the range "224.0.0.0" - "239.255.255.255"
     *
     */
    Vicon_InvalidMulticastIP = 3104,
    /**
     * @brief Connect() was called whilst already connected to a DataStream.
     *
     */
    Vicon_ClientAlreadyConnected = 3105,
    /**
     * @brief Connect() could not establish a connection to the DataStream server.
     *
     */
    Vicon_ClientConnectionFailed = 3106,
    /**
     * @brief StartTransmittingMulticast() was called when the current DataStream server was already transmitting multicast on behalf of this client.
     *
     */
    Vicon_ServerAlreadyTransmittingMulticast = 3107,
    /**
     * @brief StopTransmittingMulticast() was called when the current DataStream server was not transmitting multicast on behalf of this client.
     *
     */
    Vicon_ServerNotTransmittingMulticast = 3108,
    /**
     * @brief You have called a function which requires a connection to the DataStream server, but do not have a connection.
     *
     */
    Vicon_NotConnected = 3109,
    /**
     * @brief You have called a function which requires a frame to be fetched from the DataStream server, but do not have a frame.
     *
     */
    Vicon_NoFrame = 3110,
    /**
     * @brief An index you have passed to a function is out of range.
     *
     */
    Vicon_InvalidIndex = 3111,
    /**
     * @brief The Camera Name you passed to a function is invalid in this frame.
     *
     */
    Vicon_InvalidCameraName = 3112,
    /**
     * @brief The Subject Name you passed to a function is invalid in this frame.
     *
     */
    Vicon_InvalidSubjectName = 3113,
    /**
     * @brief The Segment Name you passed to a function is invalid in this frame.
     *
     */
    Vicon_InvalidSegmentName = 3114,
    /**
     * @brief The Marker Name you passed to a function is invalid in this frame.
     *
     */
    Vicon_InvalidMarkerName = 3115,
    /**
     * @brief The Device Name you passed to a function is invalid in this frame.
     *
     */
    Vicon_InvalidDeviceName = 3116,
    /**
     * @brief The Device Output Name you passed to a function is invalid in this frame.
     *
     */
    Vicon_InvalidDeviceOutputName = 3117,
    /**
     * @brief The Latency Sample Name you passed to a function is invalid in this frame.
     *
     */
    Vicon_InvalidLatencySampleName = 3118,
    /**
     * @brief The directions passed to SetAxisMapping() contain input which would cause two or more axes to lie along the same line, e.g. "Up" and "Down" are on the same line.
     *
     */
    Vicon_CoLinearAxes = 3119,
    /**
     * @brief The directions passed to SetAxisMapping() would result in a left-handed coordinate system. This is not supported in the SDK.
     *
     */
    Vicon_LeftHandedAxes = 3120,
    /**
     * @brief Haptic feedback is already set.
     *
     */
    Vicon_HapticAlreadySet = 3121,
    /**
     * @brief Re-timed data requested is from before the first time sample we still have
     *
     */
    Vicon_EarlyDataRequested = 3122,
    /**
     * @brief Re-timed data requested is too far into the future to be predicted
     *
     */
    Vicon_LateDataRequested = 3123,
    /**
     * @brief The method called is not valid in the current mode of operation
     *
     */
    Vicon_InvalidOperation = 3124,
    /**
     * @brief The SDK version or operating system does not support this function.
     *
     */
    Vicon_NotSupported = 3125,
    /**
     * @brief The operating system configuration changed failed.
     *
     */
    Vicon_ConfigurationFailed = 3126,
    /**
     * @brief The requested data type is not present in the stream.
     *
     */
    Vicon_NotPresent = 3127,
    /**
     * @brief The supplied argument is not within the valid range.
     *
     */
    Vicon_ArgumentOutOfRange = 3128
};


}  // namespace crf
