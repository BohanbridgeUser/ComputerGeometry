#include "Utility.h"

namespace MyCG
{
    double crossProduct(const Point_2& p1, const Point_2& p2, const Point_2& p3)
    {
        return (p2.x() - p1.x()) * (p3.y() - p2.y()) - (p2.y() - p1.y()) * (p3.x() - p2.x());
    }

    double dotProduct(const Point_2& p1, const Point_2& p2, const Point_2& p3)
    {
        return (p2.x() - p1.x()) * (p3.x() - p2.x()) + (p2.y() - p1.y()) * (p3.y() - p2.y());
    }

    double crossProduct(const Point_2& p1, const Point_2& p2)
    {
        return p1.x() * p2.y() - p1.y() * p2.x();
    }

    double Norm(const Point_2& p)
    {
        return std::sqrt(p.x() * p.x() + p.y() * p.y());
    }

    double Norm2(const Point_2& p)
    {
        return p.x() * p.x() + p.y() * p.y();
    }

    bool between(const Point_2& p1, const Point_2& p2, const Point_2& p3)
    {
        return GTZERO(dotProduct(p1,p2,p3));
    }

    bool ToLeft(const Point_2& p1, const Point_2& p2, const Point_2& p3)
    {
        double area2 = crossProduct(p1,p2,p3);
        if(GTZERO(area2)) return true;
        else if(LTZERO(area2)) return false;
        return between(p1,p2,p3);
    }

    bool ToRight(const Point_2& p1, const Point_2& p2, const Point_2& p3)
    {
        double area2 = crossProduct(p1,p2,p3);
        if(LTZERO(area2)) return true;
        else if(GTZERO(area2)) return false;
        return between(p1,p2,p3);
    }

    bool InTriangle(const Point_2& p1, const Point_2& p2, const Point_2& p3, const Point_2& p)
    {
        bool p1p2l = ToLeft(p1,p2,p), p1p2r = ToRight(p1,p2,p);
        bool p2p3l = ToLeft(p2,p3,p), p2p3r = ToRight(p2,p3,p);
        bool p3p1l = ToLeft(p3,p1,p), p3p1r = ToRight(p3,p1,p);
        return (p1p2l == p2p3l && p2p3l == p3p1l) || (p1p2r == p2p3r && p2p3r == p3p1r);
    }

    bool sort_vertex_by_xy(const Point_2& pp1, const Point_2& pp2)
    {
        if(GTEQZERO(pp1.x() - pp2.x()))
        {
            if(!(EQZERO(pp1.x() - pp2.x())) || GTZERO(pp1.y() - pp2.y()))
                return false; 
            return true;
        }
        return true;
    } 

    bool sort_vertex_by_yx(const Point_2& pp1, const Point_2& pp2)
    {
        if(GTEQZERO(pp1.y() - pp2.y()))
        {
            if(!(EQZERO(pp1.y() - pp2.y())) || GTZERO(pp1.x() - pp2.x()))
                return false; 
            return true;
        }
        return true;
    } 

    int left_than_lowest(const DataPoints_2& rpoints, int begin)
    {
        int index = begin;
        for(int i=begin+1;i<rpoints.size();++i)
        {
            if(LTZERO(rpoints[i].x() - rpoints[index].x()))
                index = i;
            else if(EQZERO(rpoints[i].x() - rpoints[index].x()) && LTZERO(rpoints[i].y() - rpoints[index].y()))
                index = i;
        }
        return index;
    }

    int find_rightest(const DataPoints_2& rpoints, int index)
    {
        int rightest = -1;
        for(int i=0;i<rpoints.size();++i)
        {
            if(i==index) continue;
            if(rightest == -1 || !ToLeft(rpoints[index],rpoints[rightest],rpoints[i]))
                rightest = i;
        }
        return rightest;
    }

    int find_rightest_index(const DataPoints_2& rpoints, std::vector<int> indices, int index)
    {
        int rightest = -1;
        for(int i=0;i<indices.size();++i)
        {
            if(i == index) continue;
            if(rightest == -1 || !ToLeft(rpoints[indices[index]],rpoints[indices[rightest]],rpoints[indices[i]]))
                rightest = i;
        }
        return rightest;
    }

    Point_2 Sort_Vertex_by_Angle::p0 = Point_2(-2e32,-2e32);

    void Sort_Vertex_by_Angle::SetP0(const DataPoints_2& rpoints)
    {
        p0 = rpoints[0];
        for(int i=0;i<rpoints.size();++i)
        {
            if(LTZERO(rpoints[i].x() - p0.x()))
                p0 = rpoints[i];
            else if(EQZERO(rpoints[i].x() - p0.x()) && LTZERO(rpoints[i].y() - p0.y()))
                p0 = rpoints[i];
        }
    }

    bool Sort_Vertex_by_Angle::operator()(const Point_2& p1, const Point_2& p2)
    {
        if(p1 == p0)
            return true;
        if(p2 == p0)
            return false;
        if(ToLeft(p0,p1,p2))
            return true;
        return false;
    }

    bool Sort_Vertex_by_XY::operator()(const Point_2& p1, const Point_2& p2)
    {
        if(GTEQZERO(p1.x() - p2.x()))
        {
            if(!(EQZERO(p1.x() - p2.x())) || GTZERO(p1.y() - p2.y()))
                return false; 
            return true;
        }
        return true;
    }

    bool Sort_Vertex_by_YX::operator()(const Point_2& p1, const Point_2& p2)
    {
        if(GTEQZERO(p1.y() - p2.y()))
        {
            if(!(EQZERO(p1.y() - p2.y())) || GTZERO(p1.x() - p2.x()))
                return false; 
            return true;
        }
        return true;
    }

    bool Sort_Vertex_by_Angle_Index::operator()(const int i1, const int i2)
    {
        if(i1 == i0)
            return true;
        if(i2 == i0)
            return false;
        if(ToLeft((*p_points)[i0],(*p_points)[i1],(*p_points)[i2]))
            return true;
        return false;
    }

    void Sort_Vertex_by_Angle_Index::SetP0(const DataPoints_2& rpoints)
    {
        p_points = &rpoints;
        for(int i=0;i<p_points->size();++i)
        {
            if((*p_points)[i].x() < (*p_points)[i0].x())
                i0 = i;
            else if((*p_points)[i].x() == (*p_points)[i0].x() && (*p_points)[i].y() < (*p_points)[i0].y())
                i0 = i;
        }
    }

    int Sort_Vertex_by_Angle_Index::i0 = 0;

    const DataPoints_2* Sort_Vertex_by_Angle_Index::p_points = nullptr;

}
