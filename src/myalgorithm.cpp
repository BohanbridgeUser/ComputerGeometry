#include "myalgorithm.h"
#include <iostream>
#include <vector>
#include <deque>
#include <stack>
#include <queue>
#include <set>
#include <map>
#include <unordered_map>
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

    bool Intersection_2::Is_Intersection_Segments(const Segment_2& rsegment1, const Segment_2& rsegment2)
    {
        Point_2 s1l = (rsegment1.source().x() < rsegment1.target().x())? rsegment1.source():rsegment1.target();
        Point_2 s1r = (rsegment1.source().x() > rsegment1.target().x())? rsegment1.source():rsegment1.target();
        Point_2 s2l = (rsegment2.source().x() < rsegment2.target().x())? rsegment2.source():rsegment2.target();
        Point_2 s2r = (rsegment2.source().x() > rsegment2.target().x())? rsegment2.source():rsegment2.target();
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

    Point_2 Intersection_2::Intersection_Segments(const Segment_2& rsegment1, const Segment_2& rsegment2)
    {
        Point_2 s1l = (rsegment1.source().x() < rsegment1.target().x())? rsegment1.source():rsegment1.target();
        Point_2 s1r = (rsegment1.source().x() > rsegment1.target().x())? rsegment1.source():rsegment1.target();
        Point_2 s2l = (rsegment2.source().x() < rsegment2.target().x())? rsegment2.source():rsegment2.target();
        Point_2 s2r = (rsegment2.source().x() > rsegment2.target().x())? rsegment2.source():rsegment2.target();
        Vector_2 va01(s1l.x(), s1l.y()), vb01(s1r.x(),s1r.y());
        Vector_2 va02(s2l.x(), s2l.y()), vb02(s2r.x(),s2r.y());
        double t = CGAL::determinant(vb01-va01,va02-va01) / (CGAL::determinant(vb01-va01,vb02-va02));
        Vector_2 result1 = va02 - t * (vb02-va02);
        return Point_2(result1.x(),result1.y());
    }

    void Intersection_2::Edge_Chasing(const DataPoints_2& rpoints, DataPoints_2& intersections,
                                      int begin1, int end1, int begin2, int end2)
    {
        
        auto fun_advance = [](int index, int begin, int end)->int{
            if(index == end) return begin;
            else return index+1;
        };
        enum InFlag{
            PIN,
            QIN,
            UNKNOWN
        };
        auto InOut = [&intersections](const Point_2& inter_p, InFlag in_flag, bool pHQ, bool qHP){
            intersections.push_back(inter_p);

            if(pHQ)
                return PIN;
            else if(qHP)
                return QIN;
            else
                return in_flag;
        };
        auto Advance1 = [&begin1, &end1, &intersections, fun_advance](int lb, int& bb, bool inside, const Point_2& inter_p){
            // if(inside)
            //     intersections.push_back(inter_p);
            bb++;
            return fun_advance(lb,begin1,end1);
        };
        auto Advance2 = [&begin2, &end2, &intersections, fun_advance](int lb, int& bb, bool inside, const Point_2& inter_p){
            // if(inside)
            //     intersections.push_back(inter_p);
            bb++;
            return fun_advance(lb,begin2,end2);
        };

        int a=begin1,  b=begin2;
        int aa = 0, bb = 0;
        int n = end1 - begin1 + 1 ,m = end2 - begin2 + 1;
        InFlag in_flag = UNKNOWN;
        bool IsFirstPoint = true;
        do{
            const Point_2& qb = rpoints[b];
            const Point_2& qe = rpoints[fun_advance(b,begin2,end2)];
            const Point_2& pb = rpoints[a];
            const Point_2& pe = rpoints[fun_advance(a,begin1,end1)];
            
            Segment_2 l1(pb, pe), l2(qb, qe);
            const auto intersection = CGAL::intersection(l1,l2);
            double determinant = CGAL::determinant(pe-pb,qe-qb);
            bool pHQ = ToLeft(qb, qe, pe), qHP = ToLeft(pb, pe, qe);
            if(intersection)
            {
                if(in_flag == UNKNOWN && IsFirstPoint)
                {
                    aa =  bb = 0;
                    IsFirstPoint = false;
                    intersections.push_back(*boost::get<Point_2>(&*intersection));
                }
                in_flag = InOut(*boost::get<Point_2>(&*intersection), in_flag, pHQ, qHP);
                a = fun_advance(a,begin1,end1);
                b = fun_advance(b,begin2,end2);
            }
            else if(GTEQZERO(determinant))
            {
                if(qHP)
                    a = Advance1(a, aa, in_flag == PIN, pe);
                else 
                    b = Advance2(b, bb, in_flag == QIN, qe);
            }
            else
            {
                if(pHQ)
                    b = Advance2(b, bb, in_flag == QIN, qe);
                else
                    a = Advance1(a, aa, in_flag == PIN, pe);
            }
        }while((aa < n || bb < m) && (aa < 2 * n ) && (bb < 2 * m));
    }

    void Intersection_2::ConvexHull_Edge_Chasing(DataPoints_2& convexhull_intersection_points, 
                                                 DataSegments_2& convexhull_intersection_segments, 
                                                 const DataPoints_2& rpoints,
                                                 const DataSegments_2& rsegments)
    {
        // All the points are sorted anticlockwise
        unsigned int convexhull_num = rpoints.size() / 30;
        std::vector<DataPoints_2> convexhulls(convexhull_num);
        DataPoints_2 points;
        for(int i=0;i<convexhull_num;++i)
        {
            DataPoints_2 temp(rpoints.begin()+i*30, rpoints.begin()+(i+1)*30);
            int LTL = left_than_lowest(temp);
            int j = LTL;
            do{
                points.push_back(temp[j]);
                j++;
                if(j==30)
                    j=0;
            }while(j!=LTL);
        }
        for(int i=0;i<points.size()-1;++i)
        {
            if(i==29)
            {
                std::cout << points[i] << " " << points[0] << std::endl;
            }
            else std::cout << points[i] << " " << points[i+1]<< std::endl;
        }
        std::cout << points[59] << " " << points[30] << std::endl;
            
        for(int i=0;i<convexhull_num;++i)
           for(int j=i+1;j<convexhull_num;++j)
                Edge_Chasing(points, convexhull_intersection_points, i*30, (i+1)*30-1, j*30, (j+1)*30-1);
        std::cout << convexhull_intersection_points.size() << std::endl;
    }

    DataPoints_2* Triangulation::cmp_Trapezoid::p_points=nullptr;
    bool Triangulation::cmp_Trapezoid::operator()(const Trapezoid& t1, const Trapezoid& t2) const
    {
        /* Triangulation */
        double x1 = 0.0, x2 = 0.0;
        if(t1.Left.end == t1.Right.end)
            x1 = 1.0/3.0 * ((*p_points)[t1.Left.begin].x() + (*p_points)[t1.Right.begin].x() + (*p_points)[t1.Right.end].x());
        else
            x1 = 0.25 * ((*p_points)[t1.Left.begin].x() + (*p_points)[t1.Right.begin].x() + (*p_points)[t1.Right.end].x() + (*p_points)[t1.Left.end].x());
        
        if(t2.Left.end == t2.Right.end)
            x2 = 1.0/3.0 * ((*p_points)[t2.Left.begin].x() + (*p_points)[t2.Right.begin].x() + (*p_points)[t2.Right.end].x());
        else
            x2 = 0.25 * ((*p_points)[t2.Left.begin].x() + (*p_points)[t2.Right.begin].x() + (*p_points)[t2.Right.end].x() + (*p_points)[t2.Left.end].x());
        
        if(LTZERO(x1-x2)) return true;
        else return false;
    }

    std::vector<std::vector<int>> Triangulation::Monotone_Polygons(DataPoints_2& rpoints, 
                                                                   std::vector<std::vector<int>>& ipolygon)
    {
        std::vector<std::vector<int>> monotone_polygons;
        /* Top to Bottom */
        for(int i=0;i<ipolygon.size();++i)
        {
            /* 1.sort all points by y coordinate */
            const std::vector<int>& polygon = ipolygon[i];
            std::vector<int> event_points;
            for(int j=0;j<polygon.size();++j)
                event_points.push_back(j);
            std::sort(event_points.begin(),event_points.end(), [&rpoints,&polygon](int i1, int i2)->bool{
                if(GTEQZERO(rpoints[polygon[i1]].y()-rpoints[polygon[i2]].y())) return true;
                else return false;
            });
            
            /* 2.monotone polygon */
            cmp_Trapezoid::p_points = &rpoints;
            std::map<Trapezoid, int, cmp_Trapezoid> status;
            int flag = 0;
            for(int j=0;j<event_points.size();++j)
            {
                int index = polygon[event_points[j]];
                int index_last = (event_points[j]==0)? polygon[polygon.size()-1]:polygon[event_points[j]-1];
                int index_next = (event_points[j]==polygon.size()-1)? polygon[0]:polygon[event_points[j]+1];
                if(rpoints[index].y() > rpoints[index_last].y() && rpoints[index].y() > rpoints[index_next].y())
                {
                    if(ToLeft(rpoints[index_last],rpoints[index],rpoints[index_next]))
                        status.insert(std::pair<Trapezoid,int>(Trapezoid{index_next,index,index_last,index},index));
                    else
                    {
                        if(!status.empty())
                        {
                            auto iter_m = status.upper_bound(Trapezoid{index,index,index,index});
                            if(iter_m == status.end())
                                --iter_m;
                            int begin_1 = iter_m->second;
                            auto iter = std::find(polygon.begin(),polygon.end(),begin_1);
                            begin_1 = iter-polygon.begin();
                            int begin_2 = event_points[j];
                            int end_1  = (event_points[j]==polygon.size()-1)? 0:event_points[j]+1;
                            int end_2  = (begin_1+1)%polygon.size();
                            std::vector<std::vector<int>> temp_polygons(2,std::vector<int>());
                            for(int k=begin_1;k!=end_1;k=(k+1)%polygon.size())
                                temp_polygons[0].push_back(polygon[k]);
                            for(int k=begin_2;k!=end_2;k=(k+1)%polygon.size())
                                temp_polygons[1].push_back(polygon[k]);
                            std::vector<std::vector<int>> temp_monotone_polygons = Monotone_Polygons(rpoints,temp_polygons);
                            for(int k=0;k<temp_monotone_polygons.size();++k)
                                monotone_polygons.push_back(temp_monotone_polygons[k]);
                            flag = 1;
                            break;
                        }
                    }
                }
                else if(rpoints[index].y() < rpoints[index_last].y() && rpoints[index].y() < rpoints[index_next].y())
                {
                    if(ToLeft(rpoints[index_last],rpoints[index],rpoints[index_next]))
                    {
                        Trapezoid T = Trapezoid{index,index_last,index,index_next};
                        status.erase(T);
                    }
                    else
                    {
                        auto iter = status.upper_bound(Trapezoid{index,index,index,index});
                        if(iter == status.end())
                            --iter;
                        while(iter->first.Left.begin != index)
                        {
                            --iter;
                            if(iter == status.end())
                                iter--;
                        }
                        const Trapezoid& T2 = iter->first;
                        const Trapezoid& T1 = (--iter)->first;
                        Trapezoid T = Trapezoid{T1.Left.begin, T1.Left.end, T2.Right.begin, T2.Right.end};
                        status.erase(T1);status.erase(T2);
                        status.insert(std::pair<Trapezoid,int>(T,index));
                    }
                }
                else if(rpoints[index].y() < rpoints[index_last].y() && rpoints[index].y() > rpoints[index_next].y())
                {
                    Trapezoid index_temp = Trapezoid{index,index,index,index};
                    auto iter = status.upper_bound(index_temp); 
                    const Trapezoid* pT1 = nullptr;
                    if(iter == status.end())
                        iter--;
                    pT1 = &(iter->first);
                    while(pT1->Left.begin != index)
                    {
                        --iter;
                        if(iter == status.end())
                            iter--;
                        pT1 = &(iter->first);
                    }
                    const Trapezoid& rT = *pT1;
                    Trapezoid T = Trapezoid{index_next, index, rT.Right.begin, rT.Right.end};
                    status.erase(rT);
                    status.insert(std::pair<Trapezoid,int>(T,index));
                }
                else if(rpoints[index].y() > rpoints[index_last].y() && rpoints[index].y() < rpoints[index_next].y())
                {
                    Trapezoid index_temp = Trapezoid{index,index,index,index};
                    auto iter = status.upper_bound(index_temp); 
                    const Trapezoid* pT1 = nullptr;
                    if(iter == status.end())
                        iter--;
                    pT1 = &(iter->first);
                    while(pT1->Right.begin != index)
                    {
                        --iter;
                        if(iter == status.end())
                            iter--;
                        pT1 = &(iter->first);
                    }
                    const Trapezoid& rT = *pT1;
                    Trapezoid T = Trapezoid{rT.Left.begin, rT.Left.end, index_last, index};
                    status.erase(rT);
                    status.insert(std::pair<Trapezoid,int>(T,index));
                }
            }
            if(flag == 0)
                monotone_polygons.push_back(polygon);
        }

        /* Bottom to Top */
        ipolygon = monotone_polygons;
        monotone_polygons = std::vector<std::vector<int>>();
        for(int i=0;i<ipolygon.size();++i)
        {
            /* 1.sort all points by y coordinate */
            const std::vector<int>& polygon = ipolygon[i];
            std::vector<int> event_points;
            for(int j=0;j<polygon.size();++j)
                event_points.push_back(j);
            std::sort(event_points.begin(),event_points.end(), [&rpoints,&polygon](int i1, int i2)->bool{
                if(LTEQZERO(rpoints[polygon[i1]].y()-rpoints[polygon[i2]].y())) return true;
                else return false;
            });
            
            /* 2.monotone polygon */
            cmp_Trapezoid::p_points = &rpoints;
            std::map<Trapezoid, int, cmp_Trapezoid> status;
            int flag = 0;
            for(int j=0;j<event_points.size();++j)
            {
                int index = polygon[event_points[j]];
                int index_last = (event_points[j]==0)? polygon[polygon.size()-1]:polygon[event_points[j]-1];
                int index_next = (event_points[j]==polygon.size()-1)? polygon[0]:polygon[event_points[j]+1];
                if(rpoints[index].y() < rpoints[index_last].y() && rpoints[index].y() < rpoints[index_next].y())
                {
                    if(ToLeft(rpoints[index_last],rpoints[index],rpoints[index_next]))
                        status.insert(std::pair<Trapezoid,int>(Trapezoid{index_last,index,index_next,index},index));
                    else
                    {
                        if(!status.empty())
                        {
                            auto iter_m = status.upper_bound(Trapezoid{index,index,index,index});
                            if(iter_m == status.end())
                                --iter_m;
                            int begin_1 = iter_m->second;
                            auto iter = std::find(polygon.begin(),polygon.end(),begin_1);
                            begin_1 = iter-polygon.begin();
                            int begin_2 = event_points[j];
                            int end_1  = (event_points[j]==polygon.size()-1)? 0:event_points[j]+1;
                            int end_2  = (begin_1+1)%polygon.size();
                            std::vector<std::vector<int>> temp_polygons(2,std::vector<int>());
                            for(int k=begin_1;k!=end_1;k=(k+1)%polygon.size())
                                temp_polygons[0].push_back(polygon[k]);
                            for(int k=begin_2;k!=end_2;k=(k+1)%polygon.size())
                                temp_polygons[1].push_back(polygon[k]);
                            std::vector<std::vector<int>> temp_monotone_polygons = Monotone_Polygons(rpoints,temp_polygons);
                            for(int k=0;k<temp_monotone_polygons.size();++k)
                                monotone_polygons.push_back(temp_monotone_polygons[k]);
                            flag = 1;
                            break;
                        }
                    }
                }
                else if(rpoints[index].y() > rpoints[index_last].y() && rpoints[index].y() > rpoints[index_next].y())
                {
                    if(ToLeft(rpoints[index_last],rpoints[index],rpoints[index_next]))
                    {
                        Trapezoid T = Trapezoid{index,index_next,index,index_last};
                        status.erase(T);
                    }
                    else
                    {
                        auto iter = status.upper_bound(Trapezoid{index,index,index,index});
                        if(iter == status.end())
                            --iter;
                        while(iter->first.Left.begin != index)
                        {
                            --iter;
                            if(iter == status.end())
                                iter--;
                        }
                        const Trapezoid& T2 = iter->first;
                        const Trapezoid& T1 = (--iter)->first;
                        Trapezoid T = Trapezoid{T1.Left.begin, T1.Left.end, T2.Right.begin, T2.Right.end};
                        status.erase(T1);status.erase(T2);
                        status.insert(std::pair<Trapezoid,int>(T,index));
                    }
                }
                else if(rpoints[index].y() < rpoints[index_last].y() && rpoints[index].y() > rpoints[index_next].y())
                {
                    Trapezoid index_temp = Trapezoid{index,index,index,index};
                    auto iter = status.upper_bound(index_temp); 
                    const Trapezoid* pT1 = nullptr;
                    if(iter == status.end())
                        iter--;
                    pT1 = &(iter->first);
                    while(pT1->Left.begin != index)
                    {
                        --iter;
                        if(iter == status.end())
                            iter--;
                        pT1 = &(iter->first);
                    }
                    const Trapezoid& rT = *pT1;
                    Trapezoid T = Trapezoid{index_last, index, rT.Right.begin, rT.Right.end};
                    status.erase(rT);
                    status.insert(std::pair<Trapezoid,int>(T,index));
                }
                else if(rpoints[index].y() > rpoints[index_last].y() && rpoints[index].y() < rpoints[index_next].y())
                {
                    Trapezoid index_temp = Trapezoid{index,index,index,index};
                    auto iter = status.upper_bound(index_temp); 
                    const Trapezoid* pT1 = nullptr;
                    if(iter == status.end())
                        iter--;
                    pT1 = &(iter->first);
                    while(pT1->Right.begin != index)
                    {
                        --iter;
                        if(iter == status.end())
                            iter--;
                        pT1 = &(iter->first);
                    }   
                    const Trapezoid& rT = *pT1;
                    Trapezoid T = Trapezoid{rT.Left.begin, rT.Left.end, index_next, index};
                    status.erase(rT);
                    status.insert(std::pair<Trapezoid,int>(T,index));
                }
            }
            if(flag == 0)
                monotone_polygons.push_back(polygon);
        }
        return monotone_polygons;
    }

    std::vector<std::vector<int>> Triangulation::Triangulation_Polygons(const DataPoints_2& rpoints, std::vector<int>& ipolygon)
    {
        std::vector<int> left_chain, right_chain;
        enum Side
        {
            Left = 0,
            Right,
            None
        };
        struct iPoint_with_Side
        {
            
            int index;
            Side left;
        };
        
        int top = 0, bottom = 0;
        for(int i=0;i<ipolygon.size();++i)
        {
            if(GTZERO(rpoints[ipolygon[i]].y() - rpoints[ipolygon[top]].y()))
                top = i;
            if(LTZERO(rpoints[ipolygon[i]].y() - rpoints[ipolygon[bottom]].y()))
                bottom = i;
        } 
        for(int i=top;i!=bottom;i=(i+1)%ipolygon.size())
            left_chain.push_back(ipolygon[i]);
        for(int i=top;i!=bottom;i=(i-1+ipolygon.size()) % ipolygon.size())
            right_chain.push_back(ipolygon[i]);
        
        std::deque<iPoint_with_Side> event_points;
        for(int i=1;i<left_chain.size();++i)
            event_points.push_back(iPoint_with_Side{left_chain[i],Left});
        for(int i=1;i<right_chain.size();++i)
            event_points.push_back(iPoint_with_Side{right_chain[i],Right});
        event_points.push_front(iPoint_with_Side{ipolygon[top], None});
        event_points.push_back(iPoint_with_Side{ipolygon[bottom], None});
        std::sort(event_points.begin(),event_points.end(),
                [&rpoints](const iPoint_with_Side& p1, const iPoint_with_Side& p2)->bool{
                    if(GTEQZERO(rpoints[p1.index].y()-rpoints[p2.index].y())) return true;
                    else return false;
                });
        
        std::vector<std::vector<int>> triangulations;
        std::deque<iPoint_with_Side> stack;
        stack.push_front(event_points.front());
        event_points.pop_front();
        stack.push_front(event_points.front());
        event_points.pop_front();
        for(auto event:event_points)
        {
            iPoint_with_Side top = stack.front();
            stack.pop_front();
            iPoint_with_Side next = stack.front();
            
            if(top.left == event.left)
            {
                if(top.left == Left)
                {
                    if(!ToLeft(rpoints[next.index],rpoints[top.index],rpoints[event.index]))
                    {
                        stack.push_front(top);
                        stack.push_front(event);
                    }
                    else
                    {
                        triangulations.push_back(std::vector<int>{event.index, top.index, next.index});
                        stack.push_front(event);
                    }
                }
                else
                {
                    if(ToLeft(rpoints[next.index], rpoints[top.index], rpoints[event.index]))
                    {
                        stack.push_front(top);
                        stack.push_front(event);
                    }
                    else
                    {
                        triangulations.push_back(std::vector<int>{event.index, top.index, next.index});
                        stack.push_front(event);
                    }
                }
            }
            else
            {
                iPoint_with_Side temp = top;
                triangulations.push_back(std::vector<int>{event.index, top.index, next.index});
                while(stack.size()>1)
                {
                    top = next;
                    stack.pop_front();
                    next = stack.front();
                    triangulations.push_back(std::vector<int>{event.index, top.index, next.index});
                }
                stack.pop_front();
                stack.push_front(temp);
                stack.push_front(event);
            }
        }
        return triangulations;
    }

    void Triangulation::Triangulation_Monotone(const DataPoints_2& rpoints, std::vector<DataPoints_2>& triangulations_points)
    {
        /* 1. monotone polygon */
        DataPoints_2 points(rpoints.begin(),rpoints.end());
        std::vector<std::vector<int>> monotone_polygons(1,std::vector<int>());
        for(int i=0;i<points.size();++i)
            monotone_polygons[0].push_back(i);
        monotone_polygons = Monotone_Polygons(points, monotone_polygons);

        /* 2. triangulation */
        std::vector<std::vector<int>> triangulations;
        for(int i=0;i<monotone_polygons.size();++i)
        {
            std::vector<std::vector<int>> temp_triangulations = Triangulation_Polygons(rpoints, monotone_polygons[i]);
            for(int j=0;j<temp_triangulations.size();++j)
                triangulations.push_back(temp_triangulations[j]);
        }

        /* 3. output */
        for(int i=0;i<triangulations.size();++i)
        {
            triangulations_points.push_back(DataPoints_2());
            for(int j=0;j<triangulations[i].size();++j)
                triangulations_points[triangulations_points.size()-1].push_back(rpoints[triangulations[i][j]]);
        }
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

    DataSegments_2 ConvexHull_2::ConvexHull_Graham_Scan_Index(const DataPoints_2& rpoints)
    {
        // 1. Preparation work
        std::vector<int> ipoints;
        for(int i=0;i<rpoints.size();++i)
            ipoints.push_back(i);
        Sort_Vertex_by_Angle_Index::SetP0(rpoints);
        std::cout << Sort_Vertex_by_Angle_Index::GetP0() << std::endl;
        std::sort(ipoints.begin(),ipoints.end(),Sort_Vertex_by_Angle_Index());
        std::cout << ipoints[0] << std::endl;
        std::cout << rpoints[ipoints[0]] << std::endl;
        std::stack<int> convexhull_points, temp_stack;
        convexhull_points.push(ipoints[0]);
        int rightest = find_rightest(rpoints,0);
        std::cout << rightest << std::endl;
        std::cout << rpoints[rightest] << std::endl;
        convexhull_points.push(rightest);
        for(int i=ipoints.size()-1;i>0;--i)
        {
            if(ipoints[i]==rightest) continue;
            temp_stack.push(ipoints[i]);
        }

        // 2. Construct convex hull
        while(!temp_stack.empty())
        {
            int i1 = convexhull_points.top();
            const Point_2* c1 = &rpoints[convexhull_points.top()];
            convexhull_points.pop();
            const Point_2 *c2 = &rpoints[convexhull_points.top()];
            convexhull_points.push(i1);
            int i2 = temp_stack.top();
            const Point_2* t1 = &rpoints[temp_stack.top()];
            while(!ToLeft(*c2,*c1,*t1))
            {
                convexhull_points.pop();
                c1 = &rpoints[convexhull_points.top()];
                int index_temp = convexhull_points.top();
                convexhull_points.pop();
                c2 = &rpoints[convexhull_points.top()];
                convexhull_points.push(index_temp);
            }
            convexhull_points.push(i2);
            temp_stack.pop();
        }

        // 3. Output
        DataSegments_2 convexhull_segments;
        int last = convexhull_points.top();
        while(convexhull_points.size()>1)
        {
            int ip1 =  convexhull_points.top();
            Point_2 tp1 = rpoints[ip1];
            convexhull_points.pop();
            int ip2 = convexhull_points.top();
            Point_2 tp2 = rpoints[ip2];
            convexhull_segments.push_back(Segment_2(tp1,tp2));
        }
        int temp_index = convexhull_points.top();
        convexhull_segments.push_back(Segment_2(rpoints[temp_index],rpoints[last]));
        return convexhull_segments;
    }

    Polyhedron Voronoi::trivalVD(const DataPoints_2& rpoints, int begin, int end, Polyhedron& rpolyhedron)
    {
        typedef typename Polyhedron::Halfedge_handle 
                                                            Halfedge_handle;
        typedef typename Polyhedron::Vertex_handle 
                                                            Vertex_handle;

        if(end - begin < 1)
            return rpolyhedron;
        else
        {
            Point_2 p1(rpoints[begin].x(), rpoints[begin].y());
            Point_2 p2(rpoints[end].x(), rpoints[end].y());
            Point_2 mid_point((p1.x()+p2.x())/2.0, (p1.y()+p2.y())/2.0);
            Line_2 line(p1,p2);
            Line_2 perpendicular_line = line.perpendicular(mid_point);
            std::vector<Halfedge_handle> halfedge_handles;
            for(auto edge_handle = rpolyhedron.edges_begin();edge_handle!=rpolyhedron.edges_end();++edge_handle)
            {
                if(edge_handle->is_border())
                    continue;

                Point_2 p1(edge_handle->vertex()->point().x(), edge_handle->vertex()->point().y());
                Point_2 p2(edge_handle->opposite()->vertex()->point().x(), edge_handle->opposite()->vertex()->point().y());
                Segment_2 edge_segment(p1,p2);
                const auto intersection = CGAL::intersection(edge_segment,perpendicular_line);
                if(intersection)
                {
                    if(const Point_2* p = boost::get<Point_2>(&*intersection))
                    {
                        Halfedge_handle halfedge_handle = rpolyhedron.split_edge(edge_handle);
                        halfedge_handle->vertex()->point() = Point_3(p->x(),p->y(),0.0);
                        halfedge_handles.push_back(halfedge_handle);
                    }
                }
            }
            rpolyhedron.split_facet(halfedge_handles[0],halfedge_handles[1]);
            return rpolyhedron;
        }     
    }

    Polyhedron Voronoi::MergeVD(const Polyhedron& rpolyhedron_left, const Polyhedron& rpolyhedron_right)
    {
        Polyhedron polyhedron;
        return polyhedron;
    }

    Polyhedron Voronoi::dacVD(const DataPoints_2& rpoints, int begin, int end, Polyhedron& rpolyhedron)
    {
        int mid = (begin + end) / 2;
        if(end - begin < 3)
            return trivalVD(rpoints, begin, end, rpolyhedron);
        return MergeVD(dacVD(rpoints, begin, mid, rpolyhedron), dacVD(rpoints, mid+1, end, rpolyhedron));
    }

    void Voronoi::Voronoi_Divide_and_Conquer(const DataPoints_2& rpoints, Polyhedron& polyhedron)
    {
        DataPoints_2 points(rpoints);
        std::sort(points.begin(),points.end(),[](const Point_2& p1, const Point_2& p2)->bool{
            if(LTZERO(p1.x()-p2.x())) return true;
            else return false;
        });

        typedef typename Polyhedron::HalfedgeDS HDS;
        typedef CGAL::Polyhedron_incremental_builder_3<HDS> Polyhedron_Builder;
        /* all zone for voronoi */
        Polyhedron_Builder Builder(polyhedron.hds(), true);
        Builder.begin_surface(4, 1);
        Builder.add_vertex(Point_3(-2.5, -2.5, 0.0));
        Builder.add_vertex(Point_3(-2.5,  2.5, 0.0));
        Builder.add_vertex(Point_3( 2.5,  2.5, 0.0));
        Builder.add_vertex(Point_3( 2.5, -2.5, 0.0));
        Builder.begin_facet();
        Builder.add_vertex_to_facet(0);
        Builder.add_vertex_to_facet(1);
        Builder.add_vertex_to_facet(2);
        Builder.add_vertex_to_facet(3);
        Builder.end_facet();
        Builder.end_surface();
    
        int mid = (points.size()-1) / 2;
        polyhedron = MergeVD(dacVD(points, 0, mid, polyhedron), dacVD(points, mid+1, points.size()-1, polyhedron));
    }

    void Voronoi::Voronoi_Sweep_Line(const DataPoints_2& rpoints, Polyhedron& polyhedron)
    {

    }

}