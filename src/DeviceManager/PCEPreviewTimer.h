#ifndef _PCEPreviewTimer
#define _PCEPreviewTimer
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEDevicePreviewer.h
//
// Author: Tingzhu Zhou
//
#include "PCEDeviceCacheImp.h"

#include <QtGui/QWidget>
#include <QtCore/QTimer.h>

typedef void (*PreviewerCallingBack)(void* lpParam);

// Forwards

class PCEPreviewTimer : public QWidget
{
	Q_OBJECT
public:
	PCEPreviewTimer(PreviewerCallingBack callBack);
	~PCEPreviewTimer();

	void						start(int msec);
	void						stop();

	void						updateBuffer(PCEDeviceCacheImp* pBuffer) { m_pCache = pBuffer; }

public slots:
	void						timer_handler();

private:
	std::shared_ptr<QTimer>		mTimer;

	PreviewerCallingBack		m_CallBack;
	PCEDeviceCacheImp*			m_pCache;

private:
	// privates these
	PCEPreviewTimer(const PCEPreviewTimer&) {}
	PCEPreviewTimer& operator=(const PCEPreviewTimer&) { return *this; }
};

#endif
