/*@author yangxt
 *@date 2016-07-03
 *本版本未添加: 检查节点的g值，如果新计算得到的路径开销比该g值低，那么要重新打开该节点（即重新放入OPEN集）
 */

 #include <iostream>
 #include <stdlib.h>
 #include <vector>
// #include <algorithm>

 class CPoint
 {
 public:
 	CPoint(int x, int y):X(x),Y(y),G(0),H(0),F(0),m_parentPoint(NULL){ };
 	~CPoint(){ };
	
	void CalcF(){
		F = G + H;
	}

	int X;
	int Y;
	int G;
	int H;
	int F;
	CPoint* m_parentPoint;
 };
 
class CAStar
{
public:
	// 构造函数
	CAStar(int array[][12])
	{
		for(int i=0; i<12; ++i)
			for(int j=0; j<12; ++j)
			{
				m_array[i][j] = array[i][j];
			}
	}

	CPoint* GetMinFPoint()
	{
		int idx = 0, valueF = -9999;
		for(int i=0; i < m_openVec.size(); ++i)
		{
			if(m_openVec[i]->F < valueF)
			{
				valueF = m_openVec[i]->F;
				idx = i;
			}
		}
		return m_openVec[idx];
	}

	bool RemoveFromOpenVec(CPoint* point)
	{
		for(POINTVEC::iterator it = m_openVec.begin(); it != m_openVec.end(); ++it)
		{
			if((*it)->X == point->X && (*it)->Y == point->Y)
			{
				m_openVec.erase(it);
				return true;
			}
		}
		return false;
	}
	
	bool canReach(int x, int y)
	{
		return 0 == m_array[x][y];
	}

	bool IsAccessiblePoint(CPoint* point, int x, int y, bool isIgnoreCorner)
	{
		if(!canReach(x, y) || isInCloseVec(x, y))
			return false;
		else
		{
			//可到达的点
			if(abs(x - point->X) + abs(y - point->Y) == 1)    // 左右上下点
				return true;
			else
			{
				if(canReach(abs(x - 1), y) && canReach(x, abs(y - 1)))   // 对角点
					return true;
				else
					return isIgnoreCorner;   //墙的边角
			}
		}
	}

	std::vector<CPoint*> GetAdjacentPoints(CPoint* point, bool isIgnoreCorner)
	{
		POINTVEC adjacentPoints;
		for(int x = point->X-1; x <= point->X+1; ++x)
			for(int y = point->Y-1; y <= point->Y+1; ++y)
			{
				if(IsAccessiblePoint(point, x, y, isIgnoreCorner))
				{
					CPoint* tmpPoint = new CPoint(x, y);
					adjacentPoints.push_back(tmpPoint);
				}
			}

		return adjacentPoints;
	}

	bool isInOpenVec(int x, int y)
	{
		for(POINTVEC::iterator it = m_openVec.begin(); it != m_openVec.end(); ++it)
		{
			if((*it)->X == x && (*it)->Y == y)
				return true;
		}
		return false;
	}

	bool isInCloseVec(int x, int y)
	{
		for(POINTVEC::iterator it = m_closeVec.begin(); it != m_closeVec.end(); ++it)
		{
			if((*it)->X == x && (*it)->Y == y)
				return true;
		}
		return false;
	}
	
	void RefreshPoint(CPoint* tmpStart, CPoint* point)
	{
		int valueG = CalcG(tmpStart, point);
		if(valueG < point->G)
		{
			point->m_parentPoint = tmpStart;
			point->G = valueG;
			point->CalcF();
		}
	}
	
	void NotFoundPoint(CPoint* tmpStart, CPoint* end, CPoint* point)
	{
		point->m_parentPoint = tmpStart;
		point->G = CalcG(tmpStart, point);
		point->G = CalcH(end, point);
		point->CalcF();
		m_openVec.push_back(point);
	}
	
	int CalcG(CPoint* start, CPoint* point)
	{
		int G = (abs(point->X - start->X) + abs(point->Y - start->Y)) == 2 ? STEP : OBLIQUE;
		int parentG = point->m_parentPoint != NULL ? point->m_parentPoint->G : 0;
		return G + parentG;
	}
	
	int CalcH(CPoint* end, CPoint* point)
	{
		int step = abs(point->X - end->X) + abs(point->Y - end->Y);
		return STEP * step;
	}

	// 搜索路径
	CPoint* FindPath(CPoint* start, CPoint* end, bool isIgnoreCorner)
	{
		m_openVec.push_back(start);
		while(0 != m_openVec.size())
		{
			CPoint* tmpStart = GetMinFPoint();   // 获取F值最小的点
			RemoveFromOpenVec(tmpStart);
			m_closeVec.push_back(tmpStart);
			
			POINTVEC adjacentPoints = GetAdjacentPoints(tmpStart, isIgnoreCorner);
			for(POINTVEC::iterator it=adjacentPoints.begin(); it != adjacentPoints.end(); ++it)
			{
				CPoint* point = *it;
				if(isInOpenVec(point->X, point->Y))   // 在开启列表
					RefreshPoint(tmpStart, point);
				//else if(inCloseVec(point))
				//{
				// 检查节点的g值，如果新计算得到的路径开销比该g值低，那么要重新打开该节点（即重新放入OPEN集）		
				//}
				else
					NotFoundPoint(tmpStart, end, point);
			}
			if(isInOpenVec(end->X, end->Y)) // 目标点已经在开启列表中
			{
				for(int i=0; i < m_openVec.size(); ++i)
				{
					if(end->X == m_openVec[i]->X && end->Y == m_openVec[i]->Y)
						return m_openVec[i];
				}
			}
		}
		return end;
	}

private:
	int m_array[12][12];
	static const int STEP = 10;
	static const int OBLIQUE = 14;

	typedef std::vector<CPoint*> POINTVEC;
	POINTVEC m_openVec;
	POINTVEC m_closeVec;
};

int main()
{
	int array[12][12] = 
	{
		{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{ 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1},
		{ 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1},
		{ 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1},
		{ 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
		{ 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
		{ 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1},
		{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
	};
	
	CAStar* pAStar = new CAStar(array);
	CPoint* start = new CPoint(1, 1);
	CPoint* end = new CPoint(6, 10);
	CPoint* point = pAStar->FindPath(start, end, false);

	while(point != NULL)
	{
		std::cout << "(" << point->X << "," << point->Y << ");" << std::endl;
		point = point->m_parentPoint;
	}
	return 0;
}


