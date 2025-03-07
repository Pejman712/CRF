/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "SqlAdapter/OdbcInterface.hpp"

namespace crf {
namespace communication {
namespace sqladapter {

SQLRETURN OdbcInterface::SQLAllocHandle(
        SQLSMALLINT HandleType,
        SQLHANDLE InputHandle,
        SQLHANDLE* OutputHandlePtr) {
    return ::SQLAllocHandle(HandleType, InputHandle, OutputHandlePtr);
}

SQLRETURN OdbcInterface::SQLSetEnvAttr(
        SQLHENV EnvironmentHandle,
        SQLINTEGER Attribute,
        SQLPOINTER ValuePtr,
        SQLINTEGER StringLength) {
    return ::SQLSetEnvAttr(EnvironmentHandle, Attribute, ValuePtr, StringLength);
}

SQLRETURN OdbcInterface::SQLConnect(
        SQLHDBC ConnectionHandle,
        SQLCHAR* ServerName,
        SQLSMALLINT NameLength1,
        SQLCHAR* UserName,
        SQLSMALLINT NameLength2,
        SQLCHAR* Authentication,
        SQLSMALLINT NameLength3) {
    return ::SQLConnect(ConnectionHandle, ServerName, NameLength1, UserName, NameLength2,
        Authentication, NameLength3);
}

SQLRETURN OdbcInterface::SQLExecDirect(
        SQLHSTMT StatementHandle,
        SQLCHAR* StatementText,
        SQLINTEGER TextLength) {
    return ::SQLExecDirect(StatementHandle, StatementText, TextLength);
}

SQLRETURN OdbcInterface::SQLNumResultCols(
        SQLHSTMT StatementHandle,
        SQLSMALLINT* ColumnCountPtr) {
    return ::SQLNumResultCols(StatementHandle, ColumnCountPtr);
}

SQLRETURN OdbcInterface::SQLColAttribute(
        SQLHSTMT StatementHandle,
        SQLUSMALLINT ColumnNumber,
        SQLUSMALLINT FieldIdentifier,
        SQLPOINTER CharacterAttributePtr,
        SQLSMALLINT BufferLength,
        SQLSMALLINT* StringLengthPtr,
        SQLLEN* NumericAttributePtr) {
    return ::SQLColAttribute(StatementHandle, ColumnNumber, FieldIdentifier, CharacterAttributePtr,
        BufferLength, StringLengthPtr, NumericAttributePtr);
}

SQLRETURN OdbcInterface::SQLBindCol(
        SQLHSTMT StatementHandle,
        SQLUSMALLINT ColumnNumber,
        SQLSMALLINT TargetType,
        SQLPOINTER TargetValuePtr,
        SQLLEN BufferLength,
        SQLLEN* StrLen_or_Ind) {
    return ::SQLBindCol(StatementHandle, ColumnNumber, TargetType, TargetValuePtr, BufferLength,
        StrLen_or_Ind);
}

SQLRETURN OdbcInterface::SQLFetchScroll(
        SQLHSTMT StatementHandle,
        SQLSMALLINT FetchOrientation,
        SQLLEN FetchOffset) {
    return ::SQLFetchScroll(StatementHandle, FetchOrientation, FetchOffset);
}

SQLRETURN OdbcInterface::SQLDisconnect(
        SQLHDBC ConnectionHandle) {
    return ::SQLDisconnect(ConnectionHandle);
}

SQLRETURN OdbcInterface::SQLFreeHandle(
        SQLSMALLINT HandleType,
        SQLHANDLE Handle) {
    return ::SQLFreeHandle(HandleType, Handle);
}

}  // namespace sqladapter
}  // namespace communication
}  // namespace crf
