#ifndef _PCEDataPtrCache
#define _PCEDataPtrCache
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEDataPtrCache.h
//
//
// Author: Tingzhu Zhou
//
#include "vector"
#include <boost/thread/mutex.hpp>

template<class DataPtrT>
class PCEDataPtrCache
{
public:
	PCEDataPtrCache() : m_capacity(200){};
	~PCEDataPtrCache()	{ clear(); }

	void clear()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		for (std::vector<DataPtrT*>::iterator iter=m_arrDataPtr.begin();iter!=m_arrDataPtr.end();++iter)
		{
			DataPtrT* pData = *iter;
			if(pData)
			{
				delete pData;
				pData = NULL;
			}
		}
		m_arrDataPtr.clear();
	}

	size_t	size()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		return m_arrDataPtr.size();
	}

	void setCapacity(size_t capacity) { m_capacity = capacity; }

	bool pushBack(DataPtrT* pData)
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if(pData && m_arrDataPtr.size() < m_capacity)
		{
			m_arrDataPtr.push_back(pData);
			return true;
		}
		return false;
	}

	DataPtrT* getDataPtr(int frameId)
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if(frameId >= 0 && frameId < m_arrDataPtr.size() )
			return m_arrDataPtr.at(frameId);

		return NULL;
	}

	DataPtrT* getFrontDataPtr()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if(!m_arrDataPtr.empty() )
			return m_arrDataPtr.front();

		return NULL;
	}

	DataPtrT* getBackDataPtr()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if(!m_arrDataPtr.empty() )
			return m_arrDataPtr.back();

		return NULL;
	}

private:
	PCEDataPtrCache (const PCEDataPtrCache&); // Disabled copy constructor
	PCEDataPtrCache& operator = (const PCEDataPtrCache&); // Disabled assignment operator
private:
	boost::mutex				m_bMutex;
	std::vector<DataPtrT*>		m_arrDataPtr;
	size_t						m_capacity;
};

#endif
