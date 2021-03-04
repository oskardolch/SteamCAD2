#ifndef _DPARSER_HPP_
#define _DPARSER_HPP_

typedef struct CDUnit
{
  char sName[32];
  char sAbbrev[8];
  double dBaseToUnit;
  char sAbbrev2[8];
  int iUnitType; // 0 unknown, 1 length, 2 angle
  char *psPos; // auxiliary value
} *PDUnit;

typedef class CDUnitList
{
private:
  int m_iDataLen;
  int m_iDataSize;
  PDUnit m_pData;
  int FindPos(int iType, int iPos);
public:
  CDUnitList();
  ~CDUnitList();
  int GetCount(int iType);
  void AddUnit(CDUnit cUnit);
  PDUnit GetUnit(int iType, int iPos);
  PDUnit FindUnit(const char *psAbbrev);
} *PDUnitList;

#define IS_LENGTH_VAL(a) (((a)==0) || ((a)==1) || ((a)==3))
#define IS_ANGLE_VAL(a) (((a)==0) || ((a)==2) || ((a)==3))

// parses UTF8 string and returns the value into pdResult
// returns:
//   -1 if the string cannot be parsed
//   0 if the return value has no units and/or dimension
//   1 if the return value is a length, in this case the distance is in base units (mm)
//   2 if the return value is an angle, in this case the angle is in base units (degree)
//   3 if the return value has no units, however it contains PI
int ParseInputString(char *psNumber, PDUnitList pUnits, double *pdResult);
const char* GetEscapeOpening(const char *psBuf);
int GetPlainMaskLen(const char *psMask);
void CopyPlainMask(char *sDest, const char *sSrc);
PDUnit GetUnitAtBuf(const char *sBuf, PDUnitList pUnits);
int ValidateMask(const char *psMask, PDUnitList pUnits);
int PreParseValue(char *psMask, PDUnitList pUnits, double dVal, double dScale,
  char *psBuf, int iBufSize);
int GuessMaskUnitType(char *psMask, PDUnitList pUnits);

#endif
