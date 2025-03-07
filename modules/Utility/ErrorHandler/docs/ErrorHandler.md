@ingroup group_error_handler

The expected type is a custom type designed to represent a container that may either produce a result or an error response. It provides a standardized way in the CRF to call functions that may fail and produce an error. Please note that this class is intended to be replaced by std::expected once it is approved and added to the C++ standard, potentially in C++23.

The main purpose of the expected type is to enable the tracing of errors and understanding where a function failed from a higher level. Previously, the returned value varied depending on the developer's choice, making it unclear what caused the error. This lack of clarity led to users not realizing that an empty or invalid return value could signify an error (e.g., using -1 as an error code for a function returning an int). Although std::optional helped make error checking clearer, it did not facilitate tracing the error back to its source.

The crf::expected type aims to behave similarly to std::optional. However, instead of returning a std::nullopt, it returns a custom error code indicating any changes made to the payload. These error codes follow HTTP standards, where codes in the 200s indicate successful results (e.g., 200 for OK).

An expected container consists of three possible values:

- Payload: The desired result.
- Code: An error code following HTTP standards.
- Detail: Additional information about the error.

### Serialization and Deserialization

This documentation provides an overview and usage instructions for converting expected into a JSON object and vice versa using the nlohmann::json library.

#### Introduction

The provided code snippet includes two functions for JSON serialization and deserialization: to_json and from_json. These functions allow converting a crf::expected<T> object to a JSON object and vice versa.

#### Sample JSON Structure

The resulting JSON object structure will depend on the contents of the expected object being serialized. However, based on the provided code snippet, a sample structure of the resulting JSON object can be represented as follows:


Example of a JSON for a type crf::expected<T>:

```json
{
  "value": T,
  "code": 204,
  "detail": 0
}
```

Following this example, if we want to understand how a crf::expected<bool> behaves, we will have:

```json
{
  "value": true,
  "code": 200,
  "detail": 0
}
```

If we imagine a function that returns a crf::expected<double> but there was an error, then you'll receive:

```json
{
  "code": 400,
  "detail": 12
}
```

Note that the values in the fields code and detail are examples and the real values inside will depend on the errors encountered.


Within these examples:
- The field value contains the return value of the function. If it's there, it's safe to assume that the function call succeeded (still, checking the code is important in case there are some caviats)
- The field code states the error encountered or (in case of success) a value of 200 meaning OK
- The field detail defaults to 0 and it's used to specify the error code received. This detail will follow the manufacturer or protocol. As an example, the field code could indicate an error in a Kinova Arm. The field detail will forward the error reported directly from Kinova, to understand it we'll have to check the Kinova documentation.
