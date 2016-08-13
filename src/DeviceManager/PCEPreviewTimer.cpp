// File: PCEPreviewTimer.cpp
//
// Author: Tingzhu Zhou
//
#include "PCEPreviewTimer.h"


PCEPreviewTimer::PCEPreviewTimer(PreviewerCallingBack callBack)
	: m_CallBack(callBack)
	, m_pCache(NULL)
{
	mTimer = std::shared_ptr<QTimer>(new QTimer(this));
	connect(mTimer.get(), SIGNAL(timeout()), this, SLOT(timer_handler()));
}

PCEPreviewTimer::~PCEPreviewTimer()
{
	mTimer->stop();
}

void PCEPreviewTimer::timer_handler()
{
	if (!m_pCache)
		return;

	//Refresh Preview
	if (m_CallBack)
	{
		(*m_CallBack)(m_pCache);
	}
}

void PCEPreviewTimer::start(int msec)
{
	if (mTimer->isActive()) {
		printf_s("Neuron timer interval changed to %d.\n", msec);
		mTimer->setInterval(msec);
	}
	else {
		printf_s("Neuron timer started with interval = %d.\n", msec);
		mTimer->start(msec);
	}
}

void PCEPreviewTimer::stop()
{
	mTimer->stop();
}
