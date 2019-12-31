#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <string>

namespace gazebo{
	/*The EnvMap representing the knowledge learned during the exploration of the robots.
	  This map object whould be the robot's public variable
	  The center of the occupancy grid corresponds to the robot's initial position*/
	class OccupancyGrid{
	public:
		OccupancyGrid();
		OccupancyGrid(const OccupancyGrid& grid);
		OccupancyGrid(int n_row, int n_col, double cell_size);
		~OccupancyGrid();
		
		int NRow();
		int NCol();
		void Reset();
		double CellSize();

		bool SetOccupancy(int r, int c, double prob);
		double GetOccupancy(int r, int c);

		
		bool SetGrid(int r, int c, bool occupied);
		bool GetGrid(int r, int c);

		std::string Serialize();

		void ShowGrid(int bot_r=-1, int bot_c=-1);
		void ShowOccupancy(int bot_r=-1, int bot_c=-1);

		OccupancyGrid& operator=(const OccupancyGrid& grid);

		std::string ToString();
	private:
		int _n_row;
		int _n_col;
		double _cell_size;
		std::vector<std::vector<bool>> _grid;
		std::vector<std::vector<double>> _occupancy;
	};
}

#endif
