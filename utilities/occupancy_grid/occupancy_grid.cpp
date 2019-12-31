#include "occupancy_grid.h"

gazebo::OccupancyGrid::OccupancyGrid(){
	this->_cell_size=1.0;  // same as robot diameter
	this->_n_row = 101;
	this->_n_col = 101;
	_grid=std::vector<std::vector<bool>>(_n_row,std::vector<bool>(_n_col,false));
	_occupancy=std::vector<std::vector<double>>(_n_row,std::vector<double>(_n_col,0.5));
}

gazebo::OccupancyGrid::OccupancyGrid(const OccupancyGrid& grid){
	this->_cell_size=grid._cell_size;  // same as robot diameter
	this->_n_row = grid._n_row;
	this->_n_col = grid._n_col;
	this->_grid=grid._grid;
	this->_occupancy=grid._occupancy;
}

gazebo::OccupancyGrid::OccupancyGrid(int n_row, int n_col, double cell_size){
	this->_cell_size=cell_size;
	this->_n_row = n_row;
	this->_n_col = n_col;
	_grid=std::vector<std::vector<bool>>(_n_row,std::vector<bool>(_n_col,false));
	_occupancy=std::vector<std::vector<double>>(_n_row,std::vector<double>(_n_col,0.5));
}

gazebo::OccupancyGrid::~OccupancyGrid(){}

gazebo::OccupancyGrid& gazebo::OccupancyGrid::operator=(const OccupancyGrid& grid){
	_grid=grid._grid;
	_occupancy=grid._occupancy;
	return *this;
}

bool gazebo::OccupancyGrid::SetOccupancy(int r, int c, double prob){
	if (r>=0 && r<_n_row && c>=0 && c<_n_col)
	{
		_occupancy[r][c]=prob;
		return true;
	}
	return false;
}

double gazebo::OccupancyGrid::GetOccupancy(int r, int c){
	if (r>=0 && r<_n_row && c>=0 && c<_n_col){
		return _occupancy[r][c];
	}
	return 0.5;  // all the space outside the grid is unknown
}

bool gazebo::OccupancyGrid::SetGrid(int r, int c, bool occupied){
	if (r>=0 && r<_n_row && c>=0 && c<_n_col)
	{
		_grid[r][c]=occupied;
		return true;
	}
	return false;
}

bool gazebo::OccupancyGrid::GetGrid(int r, int c){
	if (r>=0 && r<_n_row && c>=0 && c<_n_col){
		return _grid[r][c];
	}
	return true;  // assume all the space outside the grid is occupied
}

int gazebo::OccupancyGrid::NRow(){
	return _n_row;
}

int gazebo::OccupancyGrid::NCol(){
	return _n_col;
}

double gazebo::OccupancyGrid::CellSize(){
	return _cell_size;
}

void gazebo::OccupancyGrid::Reset(){
	_grid=std::vector<std::vector<bool>>(_n_row,std::vector<bool>(_n_col,false));
	_occupancy=std::vector<std::vector<double>>(_n_row,std::vector<double>(_n_col,0.5));
}

std::string gazebo::OccupancyGrid::Serialize(){
	std::string str="";
	for (int i = 0; i < _n_row; ++i)
	{
		for (int j = 0; j < _n_col; ++j)
		{
			if (_occupancy[i][j]!=0.5){  // the cell is known
				if (_grid[i][j])
				{
					str+=(std::to_string(i)+","+
					  std::to_string(j)+","+
					  std::to_string(_occupancy[i][j])+",");
				}else{
					str+=(std::to_string(i)+","+
					  std::to_string(j)+","+
					  std::to_string(_occupancy[i][j])+",");
				}
			}
		}
	}
	return str;
}

void gazebo::OccupancyGrid::ShowGrid(int bot_r, int bot_c){
	std::string out="";
	for (int i = _n_row-1; i>=0; i--)
	{	
		for (int j = 0; j<_n_col; j++)
		{
			//the order the printing should not be changed
			if (bot_r==i && bot_c==j)
			{
				if (_grid[i][j])
				{
					out+="%";
				}else{
					out+="#";
				}
				
			}else if (!_grid[i][j] && _occupancy[i][j]==0.5)
			{
				out+="?";
			}else if(_grid[i][j]){
				out+="@";
			}else{
				out+=" ";
			}
		}
		out+="\n";
	}
	std::cout<<out<<std::endl;
}

void gazebo::OccupancyGrid::ShowOccupancy(int bot_r, int bot_c){
	std::string out="";
	for (int i = _n_row-1; i>=0; i--)
	{	
		for (int j = 0; j<_n_col; j++)
		{
			// the order the printing should not be changed
			if (bot_r==i && bot_c==j)
			{
				out+="#";
			}else{
				out+=" "+std::to_string(_occupancy[i][j]);
			}
		}
		out+="\n";
	}
	std::cout<<out<<std::endl;
}

std::string gazebo::OccupancyGrid::ToString(){
	std::string out="";
	for (int i = _n_row-1; i>=0; i--)
	{	
		for (int j = 0; j<_n_col; j++)
		{
			out+=" "+std::to_string(_occupancy[i][j]);
		}
		out+="\n";
	}
	return out;
}


