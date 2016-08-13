///////////////////////////////////////////////////////////////////////////////
//
// PCEPointCloud.cpp
//
///////////////////////////////////////////////////////////////////////////////

#include "PCEPointCloud.h"

PCEPointCloud::PCEPointCloud()
{}

PCEPointCloud::~PCEPointCloud() {
	pntCloud.clear();
	indicesClusters.clear();
	m_validIndices.clear();
	m_firstPlayerIndices.clear();
	m_secondPlayerIndices.clear();
}

/* override */
PCEPointCloud& PCEPointCloud::operator=( const PCEPointCloud& other )
//
// Copy the geometry
//
{
	if ( &other != this ) {
		pntCloud      = other.pntCloud;
		indicesClusters = other.indicesClusters;
		m_validIndices = other.m_validIndices;
		m_firstPlayerIndices = other.m_firstPlayerIndices;
		m_secondPlayerIndices = other.m_secondPlayerIndices;
	}

	return *this;
}

void PCEPointCloud::initialize(unsigned int width, unsigned int height)
{
	pntCloud.width = width;
	pntCloud.height = height;
	pntCloud.resize(width * height);
	pntCloud.is_dense = true;
	isNormalValid = false;
}

void PCEPointCloud::setPtPosition(unsigned int w, unsigned int h, const PCEPointT& pnt)
{
	if(w < pntCloud.width && h < pntCloud.height)
		pntCloud.at(w, h) = pnt;
}

const unsigned int* PCEPointCloud::getValidIndexBuffer() const
{
	if(m_validIndices.empty())
		return NULL;
	else
		return &(m_validIndices.at(0));
}

const unsigned int* PCEPointCloud::getFirstPlayerIndexBuffer() const
{
	if(m_firstPlayerIndices.empty())
		return NULL;
	else
		return &(m_firstPlayerIndices.at(0));
}

const unsigned int* PCEPointCloud::getSecondPlayerIndexBuffer() const
{
	if(m_secondPlayerIndices.empty())
		return NULL;
	else
		return &(m_secondPlayerIndices.at(0));
}
