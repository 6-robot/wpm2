/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include "SerialCom.h"
#pragma once

class CWPM2_Driver : public CSerialCom
{
public:
    CWPM2_Driver();
    ~CWPM2_Driver();
    void Parse(unsigned char inData);
	void SetJoints(double* inPos, int* inSpeed);

	int nParseCount;
	int nRecvJointPos[7];
	int nRecvJointCurrent[7];

protected:
	unsigned char* m_SendBuf;
	unsigned char m_ParseBuf[128];
	int m_nRecvIndex;			//接收索引
	unsigned char m_lastRecv;	//上一个字符
	bool m_bFrameStart;			//帧解析开始
	int m_nFrameLength;			//帧长度

	int arPosition[7];

	void m_CalSendSum(unsigned char* pNewCmdBuf);

	void m_ParseFrame();
	void m_DisRecv();

	void m_Split2Bytes(unsigned char *inTarg, short inSrc);
	void m_Split4Bytes(unsigned char *inTarg, int inSrc);
	short m_WordFromChar(unsigned char *inBuf);
	int m_IntFromChar(unsigned char *inBuf);
};