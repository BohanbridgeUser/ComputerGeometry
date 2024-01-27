#include "myalgorithm.h"
#include <iostream>
#include <vector>
#include <stack>
#include <queue>
#include <set>
#include <algorithm>
#include <iterator>

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

    DataSegments_2 ConvexHull_2::ConvexHull_2_TriMethod(const DataPoints_2& rpoints)
    {
        // 1. Construct all triangles
        std::vector<int>    triangles;
        DataPoints_2        points=rpoints;
        for(int i=0;i<points.size();i++)
            for(int j=i+1;j<points.size();j++)
                for(int k=j+1;k<points.size();k++)
                {
                    triangles.push_back(i);
                    triangles.push_back(j);
                    triangles.push_back(k);
                }

        // 2. Test every point is in a triangle or not
        std::vector<int>    convexhull;
        for(int i=0;i<points.size();i++)
        {
            bool is_in_triangle = false;
            for(int j=0;j<triangles.size();j+=3)
            {
                if(triangles[j] != i && triangles[j+1] != i && triangles[j+2] != i &&
                    InTriangle(points[triangles[j]],points[triangles[j+1]],points[triangles[j+2]],points[i]))
                {
                    is_in_triangle = true;
                    break;
                }
            }
            if(!is_in_triangle)
                convexhull.push_back(i);
        }

        // 3. Construct convex hull
        DataSegments_2 convexhull_lines;
        DataPoints_2    convexhull_points;
        for(int i=0;i<convexhull.size();++i)
            convexhull_points.push_back(points[convexhull[i]]);
        Sort_Vertex_by_Angle::SetP0(convexhull_points);
        std::sort(convexhull_points.begin(),convexhull_points.end(),Sort_Vertex_by_Angle());   
        for(int i=0;i<convexhull.size()-1;i++)
            convexhull_lines.push_back(Segment_2(convexhull_points[i],convexhull_points[i+1]));
        convexhull_lines.push_back(Segment_2(convexhull_points[convexhull.size()-1],convexhull_points[0]));
        return convexhull_lines;
    }

    DataSegments_2 ConvexHull_2::ConvexHull_2_EE(const DataPoints_2& rpoints)
    {
        DataPoints_2        points = rpoints;
        DataSegments_2      convexhull_lines;
        int LTL = left_than_lowest(points);
        std::swap(points[0],points[LTL]);
        for(int i=0;i<points.size();++i)
            for(int j=i+1;j<points.size();++j)
            {
                bool leftfree = true, rightfree = true;
                for(int k=0;k<points.size() && (leftfree || rightfree);++k)
                {
                    if(k!=i && k != j)
                    {
                        if(ToLeft(points[i],points[j],points[k]) || ToRight(points[j],points[i],points[k]))
                            leftfree = false;
                        if(ToRight(points[i],points[j],points[k]) || ToLeft(points[j],points[i],points[k]))
                            rightfree = false;
                    }
                }
                if(leftfree || rightfree)
                    convexhull_lines.push_back(Segment_2(points[i],points[j]));
            }
        return convexhull_lines;
    }

    DataSegments_2 ConvexHull_2::ConvexHull_2_Jarvis_March(const DataPoints_2& rpoints)
    {
        DataPoints_2 points = rpoints;
        int LTL = left_than_lowest(points);
        DataSegments_2 convexhull_lines;
        int i=LTL;
        do
        {
            int rightest = -1;
            for(int j=0;j<points.size();++j)
            {
                if(i==j) continue;
                if(rightest==-1 || ToLeft(points[i],points[rightest],points[j]))
                    rightest = j;
            }
            convexhull_lines.push_back(Segment_2(points[i],points[rightest]));
            i = rightest;
        }while(LTL!=i);
        return convexhull_lines;
    }

    DataSegments_2 ConvexHull_2::ConvexHull_2_Graham_Scan(const DataPoints_2& rpoints)
    {
        // 1. Preparation work
        DataPoints_2 points = rpoints;
        Sort_Vertex_by_Angle::SetP0(points);
        std::sort(points.begin(),points.end(),Sort_Vertex_by_Angle());
        std::stack<Point_2*>  convexhull_points, temp_stack;
        convexhull_points.push(&points[left_than_lowest(points)]);
        int rightest = find_rightest(points,left_than_lowest(points));
        convexhull_points.push(&points[rightest]);
        for(int i=points.size()-1;i>0;--i)
        {
            if(i==rightest) continue;
            temp_stack.push(&points[i]);
        }

        // 2. Construct convex hull
        while(!temp_stack.empty())
        {
            Point_2* c1 = convexhull_points.top();
            convexhull_points.pop();
            Point_2 *c2 = convexhull_points.top();
            convexhull_points.push(c1);
            Point_2* t1 = temp_stack.top();
            while(!ToLeft(*c2,*c1,*t1))
            {
                convexhull_points.pop();
                c1 = convexhull_points.top();
                convexhull_points.pop();
                c2 = convexhull_points.top();
                convexhull_points.push(c1);
            }
            convexhull_points.push(t1);
            temp_stack.pop();
        }

        DataSegments_2 convexhull_segments;
        auto last = convexhull_points.top();
        while(convexhull_points.size()>1)
        {
            Point_2* tp1 = convexhull_points.top();
            convexhull_points.pop();
            Point_2* tp2 = convexhull_points.top();
            convexhull_segments.push_back(Segment_2(*tp1,*tp2));
        }
        convexhull_segments.push_back(Segment_2(*convexhull_points.top(),*last));
        return convexhull_segments;
    }

    int DAC(const DataPoints_2& rpoints, 
             std::vector<int>& convexhull_points, 
             int begin, int end)
    {
        // 0. Recursive termination condition
        if(end - begin + 1 < 6)
        {
            int LTL = begin;
            for(int i=begin+1;i<=end;++i)
            {
                if(LTZERO(rpoints[i].x() - rpoints[LTL].x()))
                    LTL = i;
                else if(EQZERO(rpoints[i].x() - rpoints[LTL].x()) && LTZERO(rpoints[i].y() - rpoints[LTL].y()))
                    LTL = i;
            }
            int i=LTL, cnt = 0;
            convexhull_points[begin+cnt++] = LTL;
            do
            {
                int rightest = -1;
                for(int j=begin;j<=end;++j)
                {
                    if(i==j) continue;
                    if(rightest==-1 || !ToLeft(rpoints[i],rpoints[rightest],rpoints[j]))
                        rightest = j;
                }
                if(rightest!=LTL) convexhull_points[begin+cnt++] = rightest;
                i = rightest;
            }while(LTL!=i);
            return cnt+begin-1;
        }       
        
        // 1. Divide
        int mid = (begin + end) / 2;
        int l_tail = DAC(rpoints,convexhull_points,begin,mid);
        int r_tail = DAC(rpoints,convexhull_points,mid+1,end);
        // 2. Merge
        // 2.1 find the rightest point in the left part and the leftest point in the right part
        //     find the leftest point in the right part and the rightest point in the left part
        int l_righest = l_tail, r_leftest = mid+1, l_leftest = begin, r_righest = r_tail;
        for(int i=begin;i<=l_tail;++i)
        {   
            if(rpoints[convexhull_points[i]].x() > rpoints[convexhull_points[l_righest]].x())
                l_righest = i;
            else if(rpoints[convexhull_points[i]].x() == rpoints[convexhull_points[l_righest]].x() &&
                    rpoints[convexhull_points[i]].y() >  rpoints[convexhull_points[l_righest]].y())
                l_righest = i;
            if(rpoints[convexhull_points[i]].x() < rpoints[convexhull_points[l_leftest]].x())
                l_leftest = i;
            else if(rpoints[convexhull_points[i]].x() == rpoints[convexhull_points[l_leftest]].x() &&
                    rpoints[convexhull_points[i]].y() <  rpoints[convexhull_points[l_leftest]].y())
                l_leftest = i;
        }
        for(int i=mid+1;i<=r_tail;++i)
        {
            if(rpoints[convexhull_points[i]].x() < rpoints[convexhull_points[r_leftest]].x())
                r_leftest = i;
            else if(rpoints[convexhull_points[i]].x() == rpoints[convexhull_points[r_leftest]].x() &&
                    rpoints[convexhull_points[i]].y() <  rpoints[convexhull_points[r_leftest]].y())
                r_leftest = i;
            if(rpoints[convexhull_points[i]].x() > rpoints[convexhull_points[r_righest]].x())
                r_righest = i;
            else if(rpoints[convexhull_points[i]].x() == rpoints[convexhull_points[r_righest]].x() &&
                    rpoints[convexhull_points[i]].y() >  rpoints[convexhull_points[r_righest]].y())
                r_righest = i;
        }

        // 2.2 Zigzag to find the tangent
        int l_t = -1, r_t = -1, l_b = -1, r_b = -1;
        auto Next_r = [=](int r_index)->int{
            if(r_index == r_tail) return mid + 1;
            return r_index + 1;
        };
        auto Last_r = [=](int r_index)->int{
            if(r_index == mid + 1) return r_tail;
            return r_index - 1;
        };
        auto Next_l = [=](int l_index)->int{
            if(l_index == l_tail) return begin;
            return l_index + 1;
        };
        auto Last_l = [=](int l_index)->int{
            if(l_index == begin) return l_tail;
            return l_index - 1;
        };
        
        int l_r = l_righest, r_l = r_leftest;
        int l_next = Next_l(l_r), l_last = Last_l(l_r),
            r_next = Next_r(r_l), r_last = Last_r(r_l);


        while(l_t == -1 || r_t == -1)
        {
            while(ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_next]]) != false ||
                  ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_last]]) != false)
            {
                r_l = r_last;
                r_next = Next_r(r_l);
                r_last = Last_r(r_l);
            }
            while(ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_next]]) != true || 
                  ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_last]]) != true)
            {
                l_r = l_next;
                l_next = Next_l(l_r);
                l_last = Last_l(l_r);
            }
            if(ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_next]])==false && 
               ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_last]])==false &&
               ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_next]])==true &&
               ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_last]])==true)
            {
                l_t = l_r;
                r_t = r_l;
            }
        }

        l_r = l_righest, r_l = r_leftest;
        r_next = Next_r(r_l), r_last = Last_r(r_l);
        l_next = Next_l(l_r), l_last = Last_l(l_r);
        while(l_b == -1 || r_b == -1)
        {
            while(ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_next]]) != true ||
                  ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_last]]) != true)
            {
                r_l = r_next;
                r_next = Next_r(r_l);
                r_last = Last_r(r_l);
            }
            while(ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_next]]) != false ||
                  ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_last]]) != false)
            {
                l_r = l_last;
                l_next = Next_l(l_r);
                l_last = Last_l(l_r);
            }
            if(ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_next]])==true &&
               ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_last]])==true &&
               ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_next]])==false &&
               ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_last]])==false)
            {
                l_b = l_r;
                r_b = r_l;
            }
        }

        // 2.3 Construct the convex hull
        std::vector<int> convexhull_temp;
        int i=l_t;
        while(i!=l_b)
        {
            convexhull_temp.push_back(convexhull_points[i]);
            i=Next_l(i);
        }
        convexhull_temp.push_back(convexhull_points[i]);
        i = r_b;
        while(i!=r_t)
        {
            convexhull_temp.push_back(convexhull_points[i]);
            i=Next_r(i);
        }     
        convexhull_temp.push_back(convexhull_points[i]);          
        
        for(int i=0;i<convexhull_temp.size();++i)
            convexhull_points[begin+i] = convexhull_temp[i];
        return convexhull_temp.size()-1+begin;
    }

    DataSegments_2 ConvexHull_2::ConvexHull_2_Divide_and_Conquer(const DataPoints_2& rpoints)
    {
        DataPoints_2       points = rpoints;
        std::vector<int>   convexhull_points(points.size());
        std::sort(points.begin(),points.end(),sort_vertex_by_xy);
        int end = DAC(points,convexhull_points,0,points.size()-1);
        DataSegments_2  convexhull_lines;
        for(int i=0;i<end;++i)
            convexhull_lines.push_back(Segment_2(points[convexhull_points[i]],points[convexhull_points[i+1]]));
        convexhull_lines.push_back(Segment_2(points[convexhull_points[end]],points[convexhull_points[0]]));
        return convexhull_lines;
    }

    bool Intersection_2::CMP_Point_LR_X::operator()(const Point_LR& p1, const Point_LR& p2)
    {
        if(GTZERO(p1.point.x() - p2.point.x())) return false;
        else return true;
    }

    bool Intersection_2::CMP_Point_LR_Y::operator()(const Point_LR& p1, const Point_LR& p2)
    {
        if(GTZERO(p1.point.y() - p2.point.y())) return false;
        else return true;
    }

    bool Intersection_2::CMP_pSegment_LR_Y::operator()(const Segment_LR* s1, const Segment_LR* s2) const
    {
        if(GTEQZERO(s1->height - s2->height)) return false;
        else return true;
    }

    bool Intersection_2::CMP_Point_LR_XY::operator()(const Point_LR& p1, const Point_LR& p2)
    {
        if(GTZERO(p1.point.x() - p2.point.x()))
        {
            if(!EQZERO(p1.point.x() - p2.point.x()) || GTZERO(p1.point.y() - p2.point.y()))
                return false;
            return true;
        }
        return true;
    }

    bool Intersection_2::CMP_Point_LR_YX::operator()(const Point_LR& p1, const Point_LR& p2)
    {
        if(GTZERO(p1.point.y() - p2.point.y()))
        {
            if(!EQZERO(p1.point.y() - p2.point.y()) || GTZERO(p1.point.x() - p2.point.x()))
                return false;
            return true;
        }
        return true;
    }

    bool Intersection_2::CMP_Point_LR_XY_Greater::operator()(const Point_LR& p1, const Point_LR& p2)
    {
        if(GTZERO(p2.point.x() - p1.point.x()))
        {
            if(!EQZERO(p1.point.x() - p2.point.x()) || GTZERO(p2.point.y() - p1.point.y()))
                return false;
            return true;
        }
        return true;
    }

    bool Intersection_2::CMP_Segment_Y_Greater::operator()(const Segment_2& s1, const Segment_2& s2)
    {
        Point_2 p1 = GTZERO(s1.source().y()-s1.target().y())? s1.source() : s1.target();
        Point_2 p2 = GTZERO(s2.source().y()-s2.target().y())? s2.source() : s2.target();
        if(GTEQZERO(p1.y()-p2.y())) return false;
        else return true;
    }

    bool Intersection_2::CMP_Segment_Y_Less::operator()(const Segment_2& s1, const Segment_2& s2)
    {
        Point_2 p1 = GTZERO(s1.source().y()-s1.target().y())? s1.source() : s1.target();
        Point_2 p2 = GTZERO(s2.source().y()-s2.target().y())? s2.source() : s2.target();
        if(LTEQZERO(p2.y()-p1.y())) return false;
        else return true;
    }

    bool Intersection_2::Is_Intersection_Segments(const Segment_LR& rsegment1, const Segment_LR& rsegment2)
    {
        Point_2& s1l = rsegment1.left->point, &s1r = rsegment1.right->point;
        Point_2& s2l = rsegment2.left->point, &s2r = rsegment2.right->point;
        /* mutex test */
        if(std::min(s1l.x(),s1r.x()) > std::max(s2l.x(),s2r.x()) || std::min(s2l.x(),s2r.x()) > std::max(s1l.x(),s1r.x()) || 
           std::min(s1l.y(),s1r.y()) > std::max(s2l.y(),s2r.y()) || std::min(s2l.y(),s2r.y()) > std::max(s1l.y(),s1r.y()))
            return false;

        /* intersection test*/
        if(ToLeft(s1l,s1r,s2l) && !ToRight(s1l,s1r,s2l) && ToRight(s1l, s1r, s2r) && !ToLeft(s1l, s1r, s2r)
            && ToLeft(s2l,s2r,s1r) && !ToRight(s2l,s2r,s1r) && ToRight(s2l, s2r, s1l) && !ToLeft(s2l, s2r, s1l))
            return true;
        else if(ToLeft(s1l,s1r,s2r) && !ToRight(s1l,s1r,s2r) && ToRight(s1l, s1r, s2l) && !ToLeft(s1l, s1r, s2l)
            && ToLeft(s2l,s2r,s1l) && !ToRight(s2l,s2r,s1l) && ToRight(s2l, s2r, s1r) && !ToLeft(s2l, s2r, s1r))
            return true;
        else 
            return false;
    }

    Point_2 Intersection_2::Intersection_Segments(const Segment_LR& rsegment1, const Segment_LR& rsegment2)
    {
        Vector_2 va01(rsegment1.left->point.x(),rsegment1.left->point.y()), vb01(rsegment1.right->point.x(),rsegment1.right->point.y());
        Vector_2 va02(rsegment2.left->point.x(),rsegment2.left->point.y()), vb02(rsegment2.right->point.x(),rsegment2.right->point.y());
        double t = CGAL::determinant(vb01-va01,va02-va01) / (CGAL::determinant(vb01-va01,vb02-va02));
        Vector_2 result1 = va02 - t * (vb02-va02);
        return Point_2(result1.x(),result1.y());
    }

    DataSegments_2 Intersection_2::Interval(const DataSegments_2& segments)
    {
        struct Point_LR
        {
            Point_2     point;
            bool        is_left;
            Segment_2*  segment = nullptr;
        };
        class Point_LR_comparator
        {
            public:
                bool operator()(const Point_LR& p1, const Point_LR& p2)
                {
                    if(LTZERO(p1.point.x() - p2.point.x()))
                        return true;
                    else
                        return false;
                }
        };

        std::vector<Point_LR>   points;
        for(int i=0;i<segments.size();++i)
        {
            Point_2 p1,p2;
            p1 = segments[i].source();
            p2 = segments[i].target();
            if(GTZERO(p1.x() - p2.x()))
            {
                points.push_back(Point_LR{p2,true});
                points.push_back(Point_LR{p1,false});
            }
            else
            {
                points.push_back(Point_LR{p1,true});
                points.push_back(Point_LR{p2,false});
            }
        }
        std::sort(points.begin(),points.end(),Point_LR_comparator());
    
        int intercnt = 0;
        std::stack<int>     left_stack;
        DataSegments_2  inter_segments;
        for(int i=0;i<points.size();++i)
        {
            if(points[i].is_left)
            {
                left_stack.push(i);
            }
            else
            {
                if(left_stack.size() == 1)
                    left_stack.pop();
                else 
                {
                    Point_2 p1(points[left_stack.top()].point.x(), intercnt * 0.1 + 0.1);
                    Point_2 p2(points[i].point.x(), intercnt * 0.1 + 0.1);
                    inter_segments.push_back(Segment_2(p1,p2));
                    left_stack.pop();
                    intercnt++;
                }
            }
        }
        return inter_segments;
    }

    DataPoints_2 Intersection_2::Intersection(const DataSegments_2& rsegments)
    {
        std::vector<Point_LR>   points_lr(rsegments.size()*2);
        std::vector<Segment_LR> segments_lr(rsegments.size());
        std::set<Segment_LR*, CMP_pSegment_LR_Y>  status;
        for(int i=0,j=0;i<rsegments.size();++i,j+=2)
        {
            Point_2 p1,p2;
            p1 = rsegments[i].point(0);
            p2 = rsegments[i].point(1);
            if(GTZERO(p1.x() - p2.x()))
            {
                points_lr[j]  =Point_LR{p2, LEFT, nullptr};
                points_lr[j+1]=Point_LR{p1, RIGHT, nullptr};
            }
            else
            {
                points_lr[j]  =Point_LR{p1, LEFT, nullptr};
                points_lr[j+1]=Point_LR{p2, RIGHT, nullptr};
            }
            segments_lr[i] = Segment_LR(&(points_lr[j]), &(points_lr[j+1]), points_lr[j].point.y());
            points_lr[j].segment = &segments_lr[i];
            points_lr[j+1].segment = &segments_lr[i];
        }

        std::priority_queue<Point_LR, std::vector<Point_LR>, CMP_Point_LR_XY_Greater>  event_queue;
        for(auto point:points_lr)
            event_queue.push(point);

        DataPoints_2    intersection_points;
        while(!event_queue.empty())
        {
            Point_LR event = event_queue.top();
            event_queue.pop();
            if(event.type == LEFT)
            {
                Segment_LR& segment = *(event.segment);
                for(auto seg:status)
                {
                    Segment_LR& segment2 = *seg;
                    if(Is_Intersection_Segments(segment,segment2))
                    {
                        Point_2 p = Intersection_Segments(segment,segment2);
                        event_queue.push(Point_LR{p, INTERSECTION,
                            GTZERO(segment.height-segment2.height)? &segment:&segment2});
                        intersection_points.push_back(p);
                    }
                }
                status.insert(&segment);
            }
            else if(event.type == INTERSECTION)
            {
                auto it = status.find(event.segment);
                auto it_prev = it;
                double height = (*it)->height;
                auto it_succ = --it;
                Segment_LR* segment = *it_prev, *segment2 = *it_succ;
                status.erase(segment);
                status.erase(segment2);
                segment->height = event.point.y()-2*1e-5;
                segment2->height = event.point.y()+2*1e-5;
                status.insert(segment);
                status.insert(segment2);
            }
            else
                status.erase(event.segment);
        }
        return intersection_points;
    }

    bool Intersection_2::Is_Monotonechain_Edge_Intersect(const DataPoints_2& chain1, int left1, int right1,
                                                         const DataPoints_2& chain2, int left2, int right2)
    {
        if(right1 - left1 == 0)
            return false;
        if(right2 - left2 == 0)
            return false;

        int mid1l = (left1 + right1) / 2, mid1r = (left1 + right1) / 2 + 1;  
        int mid2l = (left2 + right2) / 2, mid2r = (left2 + right2) / 2 + 1;
        Line_2 l1(chain1[mid1l], chain1[mid1r]), l2(chain2[mid2l], chain2[mid2r]);
        Segment_2 s1(chain1[mid1l], chain1[mid1r]), s2(chain2[mid2l], chain2[mid2r]);

        if(std::max(right1-left1, right2-left2) < 2) 
        {
            const auto intersection = CGAL::intersection(s1,s2);
            if(intersection)
            {
                if(const Point_2* p_inter_point = boost::get<Point_2>(&*intersection))
                    return true;
                else if(const Segment_2* s = boost::get<Segment_2>(&*intersection))
                {
                    std::cout << *s << std::endl;
                    return false;
                }
                else
                    return false;
            }
            else
                return false;
        }
            
        const auto intersection = CGAL::intersection(l1,l2);
        if(intersection)
        {
            if(const Point_2* p_inter_point = boost::get<Point_2>(&*intersection))
            {
                bool p_on_s1 = s1.collinear_has_on(*p_inter_point), p_on_s2 = s2.collinear_has_on(*p_inter_point);
                if(p_on_s1 || p_on_s2)
                {
                    if(p_on_s1 && p_on_s2)
                        return true;
                    else if(p_on_s1)
                    {
                        if(!ToLeft(chain2[mid2l], chain2[mid2r], *p_inter_point) && !ToRight(chain2[mid2l], chain2[mid2r], *p_inter_point))
                        {
                            if(ToLeft(chain2[mid2l], chain2[mid2r], chain2[mid2l-1]) == ToLeft(chain2[mid2l], chain2[mid2r], chain1[mid1r]))
                                return Is_Monotonechain_Edge_Intersect(chain1, mid1l, right1, chain2, left2, mid2l);
                            else
                                return Is_Monotonechain_Edge_Intersect(chain1, left1, mid1r, chain2, left2, mid2l);
                        }
                        else
                        {
                            if(ToLeft(chain2[mid2l], chain2[mid2r], chain2[mid2r+1]) == ToLeft(chain2[mid2l], chain2[mid2r], chain1[mid1r]))
                                return Is_Monotonechain_Edge_Intersect(chain1, mid1l, right1, chain2, mid2r, right2);
                            else
                                return Is_Monotonechain_Edge_Intersect(chain1, left1, mid1r, chain2, mid2r, right2);
                        }
                    }
                    else if(p_on_s2)
                    {
                        if(!ToLeft(chain1[mid1l], chain1[mid1r], *p_inter_point) && !ToRight(chain1[mid1l], chain1[mid1r], *p_inter_point))
                        {
                            if(ToLeft(chain1[mid1l], chain1[mid1r], chain1[mid1l-1]) == ToLeft(chain1[mid1l], chain1[mid1r], chain2[mid2r]))
                                return Is_Monotonechain_Edge_Intersect(chain1, left1, mid1l, chain2, mid2l, right2);
                            else
                                return Is_Monotonechain_Edge_Intersect(chain1, left1, mid1l, chain2, left2, mid2r);
                        }
                        else
                        {
                            if(ToLeft(chain1[mid1l], chain1[mid1r], chain1[mid1r+1]) == ToLeft(chain1[mid1l], chain1[mid1r], chain2[mid2r]))
                                return Is_Monotonechain_Edge_Intersect(chain1, mid1r, right1, chain2, mid2l, right2);
                            else
                                return Is_Monotonechain_Edge_Intersect(chain1, mid1r, right1,  chain2, left2, mid2r);
                        }
                    }
                }
                else 
                {
                    bool s1f = (ToLeft(chain1[mid1l], chain1[mid1r], *p_inter_point) && ToRight(chain1[mid1l], chain1[mid1r], *p_inter_point))? true:false;
                    bool s2f = (ToLeft(chain2[mid2l], chain2[mid2r], *p_inter_point) && ToRight(chain2[mid2l], chain2[mid2r], *p_inter_point))? true:false;

                    if(s1f && s2f)
                    {
                        double y1 = chain1[mid1r].y();
                        double y2 = chain2[mid2r].y();
                        if(GTZERO(y1-y2))
                            return Is_Monotonechain_Edge_Intersect(chain1, std::max(mid1l,left1+1), right1, chain2, left2, right2);
                        else if(LTZERO(y1-y2))
                            return Is_Monotonechain_Edge_Intersect(chain1, left1, right1, chain2, std::max(mid2l,left2+1), right2);
                        else
                            return Is_Monotonechain_Edge_Intersect(chain1, mid1l, right1, chain2, mid2l, right2);
                    }
                    else if((!s1f) && (!s2f))
                    {
                        double y1 = chain1[mid1l].y();
                        double y2 = chain2[mid2l].y();
                        if(GTZERO(y2-y1))
                            return Is_Monotonechain_Edge_Intersect(chain1, left1, std::min(mid1r, right1-1), chain2, left2, right2);
                        else if(LTZERO(y2-y1))
                            return Is_Monotonechain_Edge_Intersect(chain1, left1, right1, chain2, left2, std::min(mid2r, right2-1));
                        else
                            return Is_Monotonechain_Edge_Intersect(chain1, left1, mid1r, chain2, left2, mid2r);
                    }
                    else if( s1f && (!s2f) )
                        return Is_Monotonechain_Edge_Intersect(chain1, mid1l, right1, chain2, left2, mid2l);
                    else if( (!s1f) && s2f )
                        return Is_Monotonechain_Edge_Intersect(chain1, left1, mid1l, chain2, mid2l, right2);
                }
            }
            else
                return false;
        }
        else
            return false;
    }

    std::vector<std::vector<int>> Intersection_2::ConvexHull_Intersection(DataPoints_2& convexhull_intersection_points, 
                                                 DataSegments_2& convexhull_intersection_segments, 
                                                 const DataPoints_2& rpoints,
                                                 const DataSegments_2& rsegments)
    {
        /**
         * 1. Default generate 7 convexhulls that contain 30 points 
         * 2. Prepare monotonechains for intersection test
         * */
        std::vector<DataPoints_2> convexhulls(7);
        std::vector<DataPoints_2> monotonechains(14);
        std::vector<std::vector<int>> intersections(7,std::vector<int>(7,0));
        unsigned int convexhull_num = rpoints.size() / 30;
        for(int i=0;i<convexhull_num;++i)
        {
            /* The Generation ConvexHull points are sorted anticlockwise */
            for(int begin=i*30;begin<30*(i+1);++begin)
                convexhulls[i].push_back(rpoints[begin]);     

            auto max = std::max_element(convexhulls[i].begin(),convexhulls[i].end(),
                                        [](const Point_2& p1, const Point_2& p2)->bool{
                                            if(GTEQZERO(p1.y() - p2.y())) return false;
                                            else return true;
                                        });
            auto min = std::min_element(convexhulls[i].begin(),convexhulls[i].end(),
                                        [](const Point_2& p1, const Point_2& p2)->bool{
                                            if(LTEQZERO(p2.y() - p1.y())) return false;
                                            else return true;
                                        });

            /* monotonechain is ordered in left-right order */
            auto iter = max;
            monotonechains[2*i].emplace_back(2e31,(*iter).y());
            do
            {
                monotonechains[2*i].push_back(*iter);
                iter++;
                if(iter == convexhulls[i].end())
                    iter = convexhulls[i].begin();
            } while (iter!=min);
            monotonechains[2*i].emplace_back(*min);
            monotonechains[2*i].emplace_back(2e31,(*iter).y());
            // std::cout << "monotonechain size :" << monotonechains[2*i].size() << std::endl;
            // for(auto mono:monotonechains[2*i])
            //     std::cout << mono << std::endl;

            iter = max;
            monotonechains[2*i+1].emplace_back(-2e31,(*iter).y());
            do{
                monotonechains[2*i+1].push_back(*iter);
                if(iter==convexhulls[i].begin())
                    iter = convexhulls[i].end()-1;
                else
                    iter--;
            }while(iter!=min);
            monotonechains[2*i+1].emplace_back(*min);
            monotonechains[2*i+1].emplace_back(-2e31,(*iter).y());
            // std::cout << "monotonechain size :" << monotonechains[2*i+1].size() << std::endl;
            // for(auto mono:monotonechains[2*i+1])
            //     std::cout << mono << std::endl;
        }  
        std::cout << "monotonechains prepared" << std::endl;
        
        /**
         * 3. Intersection test
         * */
        for(int i=0;i<convexhull_num;++i)
        {
            for(int j=i+1;j<convexhull_num;++j)
            {
                if(i==j) continue;
                if(Is_Monotonechain_Edge_Intersect(monotonechains[2*i],0,monotonechains[2*i].size()-1,
                                                   monotonechains[2*j+1],0,monotonechains[2*j+1].size()-1) &&
                   Is_Monotonechain_Edge_Intersect(monotonechains[2*i+1],0,monotonechains[2*i+1].size()-1,
                                                   monotonechains[2*j],0,monotonechains[2*j].size()-1))
                {
                    intersections[i][j] = 1;
                    std::cout << "Monotonechain " << i << " and " << j << " intersect" << std::endl;
                }
                else
                {
                    intersections[i][j] = 0;
                    std::cout << "Monotonechain " << i << " and " << j << " non-intersect" << std::endl;
                }
            }
        }
        return intersections;
    }

}