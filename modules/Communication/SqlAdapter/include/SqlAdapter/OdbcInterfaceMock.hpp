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

class OdbcInterfaceMock : public IOdbcInterface {
 public:
  MOCK_METHOD3(SQLAllocHandle,
      SQLRETURN(SQLSMALLINT HandleType, SQLHANDLE InputHandle, SQLHANDLE* OutputHandlePtr));
  MOCK_METHOD4(SQLSetEnvAttr,
      SQLRETURN(SQLHENV EnvironmentHandle, SQLINTEGER Attribute, SQLPOINTER ValuePtr,
        SQLINTEGER StringLength));
  MOCK_METHOD7(SQLConnect,
      SQLRETURN(SQLHDBC ConnectionHandle, SQLCHAR* ServerName, SQLSMALLINT NameLength1,
        SQLCHAR* UserName, SQLSMALLINT NameLength2, SQLCHAR* Authentication,
        SQLSMALLINT NameLength3));
  MOCK_METHOD3(SQLExecDirect,
      SQLRETURN(SQLHSTMT StatementHandle, SQLCHAR* StatementText, SQLINTEGER TextLength));
  MOCK_METHOD2(SQLNumResultCols,
      SQLRETURN(SQLHSTMT StatementHandle, SQLSMALLINT* ColumnCountPtr));
  MOCK_METHOD7(SQLColAttribute,
      SQLRETURN(SQLHSTMT StatementHandle, SQLUSMALLINT ColumnNumber, SQLUSMALLINT FieldIdentifier,
        SQLPOINTER CharacterAttributePtr, SQLSMALLINT BufferLength, SQLSMALLINT* StringLengthPtr,
        SQLLEN* NumericAttributePtr));
  MOCK_METHOD6(SQLBindCol,
      SQLRETURN(SQLHSTMT StatementHandle, SQLUSMALLINT ColumnNumber, SQLSMALLINT TargetType,
        SQLPOINTER TargetValuePtr, SQLLEN BufferLength, SQLLEN* StrLen_or_Ind));
  MOCK_METHOD3(SQLFetchScroll,
      SQLRETURN(SQLHSTMT StatementHandle, SQLSMALLINT FetchOrientation, SQLLEN FetchOffset));
  MOCK_METHOD1(SQLDisconnect,
      SQLRETURN(SQLHDBC ConnectionHandle));
  MOCK_METHOD2(SQLFreeHandle,
      SQLRETURN(SQLSMALLINT HandleType, SQLHANDLE Handle));
};

}  // namespace sqladapter
}  // namespace communication
}  // namespace crf
