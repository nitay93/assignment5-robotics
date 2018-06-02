//
// Created by t-idkess on 18-Mar-18.
//

#ifndef INC_5_CGAL_DEFINES_H
#define INC_5_CGAL_DEFINES_H

#include <CGAL/Gmpq.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/General_polygon_set_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Gps_segment_traits_2.h>
#include <CGAL/Arr_vertical_decomposition_2.h>
#include <CGAL/Boolean_set_operations_2/Gps_default_dcel.h>

typedef typename CGAL::Gmpq Number_type;
typedef typename CGAL::Cartesian<Number_type> Kernel;
typedef typename Kernel::FT FT;
typedef typename Kernel::Point_2 Point_2;
typedef typename Kernel::Segment_2 Segment_2;
typedef typename Kernel::Vector_2 Vector_2;
typedef typename Kernel::Line_2 Line_2;
typedef typename CGAL::Polygon_2<Kernel> Polygon_2;
typedef typename CGAL::Gps_segment_traits_2< Kernel> Gps_traits_2;
typedef typename CGAL::Arrangement_2<Gps_traits_2, CGAL::Gps_default_dcel<Gps_traits_2>> Arrangement_2;
typedef typename CGAL::General_polygon_set_2<Gps_traits_2> Polygon_set_2;
typedef typename Arrangement_2::Vertex_const_handle Arr2_Vertex;
typedef typename Arrangement_2::Halfedge_const_handle Arr2_hEdge;
#endif //INC_5_CGAL_DEFINES_H