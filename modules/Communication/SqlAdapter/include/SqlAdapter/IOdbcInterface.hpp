#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

/*
 * WARNING!!!
 *
 * Some of the C definitions or macros in sql.h header might conflict with fmt (but not only)
 * library. I am not sure yet how to permanently solve this problem, so as a workaround
 * it is required to include this header after other C++ headers.
 *
 * Since sql.h is pure C code I cannot forward declare the types used in the functions
 * signatures.
 */
#include <sql.h>

namespace crf {
namespace communication {
namespace sqladapter {

class IOdbcInterface {
 public:
    virtual ~IOdbcInterface() = default;
    virtual SQLRETURN SQLAllocHandle(
        SQLSMALLINT HandleType,
        SQLHANDLE InputHandle,
        SQLHANDLE* OutputHandlePtr) = 0;
    virtual SQLRETURN SQLSetEnvAttr(
        SQLHENV EnvironmentHandle,
        SQLINTEGER Attribute,
        SQLPOINTER ValuePtr,
        SQLINTEGER StringLength) = 0;
    virtual SQLRETURN SQLConnect(
        SQLHDBC ConnectionHandle,
        SQLCHAR* ServerName,
        SQLSMALLINT NameLength1,
        SQLCHAR* UserName,
        SQLSMALLINT NameLength2,
        SQLCHAR* Authentication,
        SQLSMALLINT NameLength3) = 0;
    virtual SQLRETURN SQLExecDirect(
        SQLHSTMT StatementHandle,
        SQLCHAR* StatementText,
        SQLINTEGER TextLength) = 0;
    virtual SQLRETURN SQLNumResultCols(
        SQLHSTMT StatementHandle,
        SQLSMALLINT* ColumnCountPtr) = 0;
    virtual SQLRETURN SQLColAttribute(
        SQLHSTMT StatementHandle,
        SQLUSMALLINT ColumnNumber,
        SQLUSMALLINT FieldIdentifier,
        SQLPOINTER CharacterAttributePtr,
        SQLSMALLINT BufferLength,
        SQLSMALLINT* StringLengthPtr,
        SQLLEN* NumericAttributePtr) = 0;
    virtual SQLRETURN SQLBindCol(
        SQLHSTMT StatementHandle,
        SQLUSMALLINT ColumnNumber,
        SQLSMALLINT TargetType,
        SQLPOINTER TargetValuePtr,
        SQLLEN BufferLength,
        SQLLEN* StrLen_or_Ind) = 0;
    virtual SQLRETURN SQLFetchScroll(
        SQLHSTMT StatementHandle,
        SQLSMALLINT FetchOrientation,
        SQLLEN FetchOffset) = 0;
    virtual SQLRETURN SQLDisconnect(
        SQLHDBC ConnectionHandle) = 0;
    virtual SQLRETURN SQLFreeHandle(
        SQLSMALLINT HandleType,
        SQLHANDLE Handle) = 0;
};

}  // namespace sqladapter
}  // namespace communication
}  // namespace crf
