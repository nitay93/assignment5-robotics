#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include "CGAL_defines.h"

using namespace std;

struct Trapezoid {
//	Trapezoid left_neighbor;
//	Trapezoid right_neighbor;
	Segment_2 top_edge;
	Segment_2 bottom_edge;
};

Point_2 loadPoint_2(std::ifstream &is) {
    Kernel::FT x, y;
    is >> x >> y;
    Point_2 point(x, y);
    return point;
}

Polygon_2 loadPolygon(ifstream &is) {
    size_t polygon_size = 0;
    is >> polygon_size;
    Polygon_2 ret;
    while (polygon_size--)
        ret.push_back(loadPoint_2(is));
    CGAL::Orientation orient = ret.orientation();
    if (CGAL::COUNTERCLOCKWISE == orient)
        ret.reverse_orientation();
    return ret;
}

vector<Polygon_2> loadPolygons(ifstream &is) {
    size_t number_of_polygons = 0;
    is >> number_of_polygons;
    vector<Polygon_2> ret;
    while (number_of_polygons--)
        ret.push_back(loadPolygon(is));
    return ret;
}

vector<pair<Point_2, Point_2>>
findPath(const Point_2 &start1, const Point_2 &end1, const Point_2 &start2, const Point_2 &end2,
         const Polygon_2 &outer_obstacle, vector<Polygon_2> &obstacles) {

	int obstacles_size = obstacles.size();

	// add all obstacles to polygon set
	Polygon_set_2 polygon_set;
	for(int i = 0; i < obstacles_size; i++) {
		polygon_set.join(obstacles.at(i));
	}



	// get free space polygon set
//	polygon_set.complement();

	// create an arrangement from the polygon set
	Arrangement_2 free_space_arrangement = polygon_set.arrangement();

	for (auto i=outer_obstacle.edges_begin(); i!=outer_obstacle.edges_end(); i++) {
		Segment_2 addSeg(i->point(0),i->point(1));
		CGAL::insert(free_space_arrangement,addSeg);
	}


	cout<<"got here1"<<endl;

	//Trapezoidal decomposition
	typedef std::list<std::pair<Arrangement_2::Vertex_const_handle,
	                              std::pair<CGAL::Object, CGAL::Object> > >
	Vert_decomp_list;
	Vert_decomp_list vd_list;
	CGAL::decompose(free_space_arrangement, std::back_inserter(vd_list));

	std::list<Trapezoid> trapezoids;


	Arr2_Vertex vert;
	Arr2_hEdge edge;
	Vector_2 horizontalVec = Point_2(0,1)-Point_2(0,0);
	Line_2 line;
	Segment_2 seg;
	list<Segment_2> segList;

	for(Vert_decomp_list::iterator it = vd_list.begin(); it != vd_list.end(); ++it) {
//		cout << it->first << endl;
//		cout << it->first. << endl;
		Point_2 p = it->first->point();
		double x = p.x().to_double();
		double y = p.y().to_double();

		//check upper element
		//TODO: wrap all polygons with a bounding box slightly larger.
		if (assign(edge,it->second.first)) { //if upper element is non fictitious half-edge
			if (!edge->is_fictitious()) {
				cout<<"(x,y): "<<it->first->point()<<" upper edge points: "<< edge->source()->point()<<" "<<edge->target()->point()<<endl;
				//todo connect vertex to edge with vertical segment
				line = Line_2(it->first->point(),horizontalVec);
				seg = Segment_2(edge->source()->point(),edge->target()->point());
				auto result = (CGAL::intersection(line,seg));
				cout<<"got here3"<<endl;
				Point_2* p = boost::get<Point_2 >(&*(result));
				cout<<"got here4"<<endl;
				Segment_2 addSeg(*p,it->first->point());
				segList.push_back(addSeg);
	//			CGAL::insert(free_space_arrangement,addSeg);
			}
		}
		if (assign(vert,it->second.first)) { //if upper element is non fictitious half-edge
			cout<<"(x,y): "<<it->first->point()<<" upper point: "<< vert->point()<<endl;
				Segment_2 addSeg(vert->point(),it->first->point());
				segList.push_back(addSeg);
	//			 CGAL::insert(free_space_arrangement,addSeg);

			}

		//check lower elements
		if (assign(edge,it->second.second)) { //if upper element is non fictitious half-edge
			if (!edge->is_fictitious()) {
				cout<<"(x,y): "<<it->first->point()<<" lower edge points: "<< edge->source()->point()<<" "<<edge->target()->point()<<endl;
				//todo connect vertex to edge with vertical segment
				line = Line_2(it->first->point(),horizontalVec);
				seg = Segment_2(edge->source()->point(),edge->target()->point());
				auto result = (CGAL::intersection(line,seg));
				Point_2* p = boost::get<Point_2 >(&*(result));
				Segment_2 addSeg(*p,it->first->point());
				segList.push_back(addSeg);
	//			CGAL::insert(free_space_arrangement,addSeg);
			}
		}
		if (assign(vert,it->second.second)) { //if upper element is non fictitious half-edge
			cout<<"(x,y): "<<it->first->point()<<" lower point: "<< vert->point()<<endl;
				Segment_2 addSeg(vert->point(),it->first->point());
				segList.push_back(addSeg);
	//			 CGAL::insert(free_space_arrangement,addSeg);

			}
		}

	for (auto i=segList.begin(); i!=segList.end(); i++) {
		CGAL::insert(free_space_arrangement,*i);
	}
/*
	//output mesh structure using ipe
	std::ofstream myFile;
	std::ifstream Template;
	 std::string line2;
	Template.open("ipe2.xml");
	myFile.open("Ipe.xml");


	while (std::getline(Template,line2)) {
		myFile <<line2<<"\n";
	}

	myFile << "<page>\n";

	for (auto i=free_space_arrangement.vertices_begin(); i!=free_space_arrangement.vertices_end(); i++) {
	myFile << "<use name=\"mark/disk(sx)\" " << "pos= \"" << i->point().x().to_double() << " " << i->point().y().to_double() << "\" size=\"normal\" stroke=\"black\"/>\n";
	}

	for (auto i = free_space_arrangement.edges_begin(); i!=free_space_arrangement.edges_end(); i++) {

	Point_2 p1 = i->source()->point();

	Point_2 p2 = i->target()->point();

	myFile << "<path stroke = \"black\"> \n"  << p1.x().to_double() <<" "<< p1.y().to_double() <<" m \n" << p2.x().to_double() <<" "<<p2.y().to_double() << " l \n" << "</path> \n";


	}

	myFile << "</page>\n";
	myFile << "</ipe>\n";
	myFile.close();

*/
		//convert triplets to trapezoid
		//go over all faces created in the decomposition
		//go over the trapezoids
//int counter=0;
//	for (auto i=free_space_arrangement.edges_begin(); i!=free_space_arrangement.edges_end(); i++) {
//		counter++;
//	}
//
//	cout<<counter<<endl;
//
//
//
//	 auto i = free_space_arrangement.faces_begin();
//
//	 while (!i->has_outer_ccb()) {
//		 i++;
//	 }
//
//
//	 Arrangement_2::Ccb_halfedge_circulator outerCCb = i->outer_ccb();
//
//	 cout<<"face circulator"<<endl;
//	 auto j = outerCCb;
//
//	 do {
//		 cout<<j->target()->point()<<" "<<j->source()->point()<<endl;
//		 j++;
//
//	 }	while (j!=outerCCb);



	return vector<pair<Point_2, Point_2>>();
}

int main(int argc, char *argv[]) {
    if (argc != 4) {
        cerr << "[USAGE]: inputRobots inputObstacles outputFile" << endl;
        return 1;
    }

    ifstream inputRobotsFile(argv[1]), inputObstaclesFile(argv[2]);
    if (!inputRobotsFile.is_open() || !inputObstaclesFile.is_open()) {
        if (!inputRobotsFile.is_open()) cerr << "ERROR: Couldn't open file: " << argv[1] << endl;
        if (!inputObstaclesFile.is_open()) cerr << "ERROR: Couldn't open file: " << argv[2] << endl;
        return -1;
    }

    auto startPoint1 = loadPoint_2(inputRobotsFile);
    auto endPoint1 = loadPoint_2(inputRobotsFile);
    auto startPoint2 = loadPoint_2(inputRobotsFile);
    auto endPoint2 = loadPoint_2(inputRobotsFile);
    inputRobotsFile.close();

    auto outer_obstacle = loadPolygon(inputObstaclesFile);
    auto obstacles = loadPolygons(inputObstaclesFile);
    inputObstaclesFile.close();

    boost::timer timer;
    auto result = findPath(startPoint1, endPoint1, startPoint2, endPoint2, outer_obstacle, obstacles);
    auto secs = timer.elapsed();
    cout << "Path created:      " << secs << " secs" << endl;

    ofstream outputFile;
    outputFile.open(argv[3]);
    if (!outputFile.is_open()) {
        cerr << "ERROR: Couldn't open file: " << argv[3] << endl;
        return -1;
    }
    outputFile << result.size() << endl;
    for (auto &p : result) {
        outputFile << p.first.x().to_double() << " " << p.first.y().to_double() << " " << p.second.x().to_double()
                   << " " << p.second.y().to_double() << endl;
    }
    outputFile.close();
    return 0;
}
