/*****************************************************************************
*	Name: EcatMasterBase.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CEcatMasterBase class.
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef _ECAT_MASTER_BASE_
#define _ECAT_MASTER_BASE_

#include "EcatCommon.h"
#include "EcatSlaveBase.h"
#include <vector>
#include <memory>

class CEcatMasterBase
{

public:
	CEcatMasterBase();
	~CEcatMasterBase();

private:
	ECAT_MASTER_STATE	GetMasterStateSt	(	);
	ECAT_DOMAIN_STATE	GetDomainStateSt	(ECAT_COMM_DIR);
	void				UpdateMasterStateSt	(	);
	void				UpdateDomainStateSt	(ECAT_COMM_DIR);

protected:
	stEcatMasterDesc				m_stEcatMasterDesc;
	std::vector<CEcatSlaveBase*>	m_vEcatSlaves;
	virtual BOOL	InitSlaves				(	);
	void			UpdateAppTime			(UINT64);	 

public:
	virtual	BOOL	Init					(UINT32, BOOL abIsEnableDC=FALSE);  // cycletime in nanoseconds
	virtual	void	DeInit					(	);
	virtual void	DeInitSlaves			(	);

	virtual void	AddSlave				(CEcatSlaveBase* acEcatSlaveBase);

	UINT32			GetCycleTime			(	){return m_unCycleTime;};
	BOOL			IsDCEnabled				(	){return m_bIsDCEnabled;};


	virtual void	ReadSlaves				(	);
	virtual void	WriteSlaves				(UINT64);


	/* Master - Domain Status */
	ECAT_STATE_MACH	GetMasterState			(	);
	ECAT_STATE_MACH	GetSlaveState			(	);
	BOOL			IsMasterOp				(	);
	BOOL			IsAllSlavesOp			(	);

	ECAT_WC			GetWorkingCounter		(ECAT_COMM_DIR aeInOut = eInput);
	ECAT_WC_STATE	GetDomainState			(ECAT_COMM_DIR aeInOut =eInput);
	BOOL			IsDomainOp				(ECAT_COMM_DIR aeInOut =eInput);

	/* no. of slaves in the vector */
	unsigned int	GetSlaveCnt				(	){return (unsigned int)(m_vEcatSlaves.size());}
	unsigned int	GetRespSlaveNo			(	);  // no. of responding slaves
	BOOL			IsLinkUp				(	);  // true if at least one Ethernet link is connected

protected:
	BOOL				m_bInit;
	UINT32				m_unCycleTime;
	BOOL				m_bIsDCEnabled;
	ECAT_MASTER_STATE	m_stMasterState;
	ECAT_DOMAIN_STATE	m_stDomainInState;
	ECAT_DOMAIN_STATE	m_stDomainOutState;

}; //CEcatMasterBase

typedef CEcatMasterBase CEcatMaster;

#endif //_ECAT_MASTER_BASE_