#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "SqlAdapter/IOdbcInterface.hpp"

namespace crf {
namespace communication {
namespace sqladapter {

class OdbcInterface: public IOdbcInterface {
 public:
    ~OdbcInterface() override = default;
    SQLRETURN SQLAllocHandle(
        SQLSMALLINT HandleType,
        SQLHANDLE InputHandle,
        SQLHANDLE* OutputHandlePtr) override;
    SQLRETURN SQLSetEnvAttr(
        SQLHENV EnvironmentHandle,
        SQLINTEGER Attribute,
        SQLPOINTER ValuePtr,
        SQLINTEGER StringLength) override;
    SQLRETURN SQLConnect(
        SQLHDBC ConnectionHandle,
        SQLCHAR* ServerName,
        SQLSMALLINT NameLength1,
        SQLCHAR* UserName,
        SQLSMALLINT NameLength2,
        SQLCHAR* Authentication,
        SQLSMALLINT NameLength3) override;
    SQLRETURN SQLExecDirect(
        SQLHSTMT StatementHandle,
        SQLCHAR* StatementText,
        SQLINTEGER TextLength) override;
    SQLRETURN SQLNumResultCols(
        SQLHSTMT StatementHandle,
        SQLSMALLINT* ColumnCountPtr) override;
    SQLRETURN SQLColAttribute(
        SQLHSTMT StatementHandle,
        SQLUSMALLINT ColumnNumber,
        SQLUSMALLINT FieldIdentifier,
        SQLPOINTER CharacterAttributePtr,
        SQLSMALLINT BufferLength,
        SQLSMALLINT* StringLengthPtr,
        SQLLEN* NumericAttributePtr) override;
    SQLRETURN SQLBindCol(
        SQLHSTMT StatementHandle,
        SQLUSMALLINT ColumnNumber,
        SQLSMALLINT TargetType,
        SQLPOINTER TargetValuePtr,
        SQLLEN BufferLength,
        SQLLEN* StrLen_or_Ind) override;
    SQLRETURN SQLFetchScroll(
        SQLHSTMT StatementHandle,
        SQLSMALLINT FetchOrientation,
        SQLLEN FetchOffset) override;
    SQLRETURN SQLDisconnect(
        SQLHDBC ConnectionHandle) override;
    SQLRETURN SQLFreeHandle(
        SQLSMALLINT HandleType,
        SQLHANDLE Handle) override;
};

}  // namespace sqladapter
}  // namespace communication
}  // namespace crf
