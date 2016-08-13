#ifndef _PCEPointCloud
#define _PCEPointCloud
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEPointCloud.h
//
// \brief PCEPointCloud represents the base class for storing collections of 3D points.
//
// Author: Tingzhu Zhou
//

#include <pcl/common/common.h>


/** @brief PCL point object */
typedef pcl::PointXYZRGBNormalUV			PCEPointT;

/** @brief PCL Point cloud object */
typedef pcl::PointCloud<PCEPointT>		PointCloudT;

typedef std::vector<pcl::PointIndices>	IndicesClusterT;

class PCEPointCloud
{
public:
	PCEPointCloud();
	~PCEPointCloud();
	PCEPointCloud& operator=( const PCEPointCloud& );

	void	initialize(unsigned int width, unsigned int height);
	void	setPtPosition(unsigned int w, unsigned int h, const PCEPointT& pnt);
	size_t	getCloudSize() const { return pntCloud.size(); }

	// ValidIndex
	void			pushValidIndex(unsigned int id) { m_validIndices.push_back(id); }
	unsigned int	getValidIndex(int id) const { return m_validIndices.at(id); }
	void			resizeValidIndex(size_t indexSize) { m_validIndices.resize(indexSize); }
	const unsigned int*	getValidIndexBuffer() const;
	size_t			getValidIndexSize() const { return m_validIndices.size(); }
	
	// PlayerIndex
	void			pushFirstPlayerIndex(unsigned int id){ m_firstPlayerIndices.push_back(id);	}
	unsigned int	getFirstPlayerIndex(int id) const { return m_firstPlayerIndices.at(id); }
	void			resizeFirstPlayerIndex(size_t indexSize) { m_firstPlayerIndices.resize(indexSize); }
	const unsigned int*	getFirstPlayerIndexBuffer() const;
	size_t			getFirstPlayerIndexSize() const { return m_firstPlayerIndices.size(); }
	void			pushSecondPlayerIndex(unsigned int id){ m_secondPlayerIndices.push_back(id); }
	unsigned int	getSecondPlayerIndex(int id) const { return m_secondPlayerIndices.at(id); }
	void			resizeSecondPlayerIndex(size_t indexSize) { m_secondPlayerIndices.resize(indexSize); }
	const unsigned int*	getSecondPlayerIndexBuffer() const;
	size_t			getSecondPlayerIndexSize() const { return m_secondPlayerIndices.size(); }
	
public:
	PointCloudT					pntCloud;
	IndicesClusterT				indicesClusters;

	std::vector<unsigned int>	m_validIndices;
	std::vector<unsigned int>	m_firstPlayerIndices;
	std::vector<unsigned int>	m_secondPlayerIndices;

	bool						isNormalValid;
};

#endif /* _PCEPointCloud */
