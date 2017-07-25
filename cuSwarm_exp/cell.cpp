#include "cell.h"

Cell::Cell() {
  min_x_ = 0.0;
  max_y_ = 0.0;
  min_x_ = 0.0;
  max_y_ = 0.0;
  is_left_explored_ = false;
  is_right_explored_ = false;
  is_ceiling_explored_ = false;
  is_floor_explored_ = false;
  // default empty std::vector<std::tuple<float2,float2> > placeholder_
  completely_explored_ = false;
}

Cell::Cell(float min_x, float max_x, float min_y, float max_y, 
    bool is_left_explored, bool is_right_explored, 
    bool is_ceiling_explored, bool is_floor_explored)
  : min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y),
    is_left_explored_(is_left_explored),
    is_right_explored_(is_right_explored),
    is_ceiling_explored_(is_ceiling_explored),
    is_floor_explored_(is_floor_explored) {

  // default empty std::vector<std::tuple<float2,float2> > placeholder_
  completely_explored_ = false;
}

float Cell::GetMinX() const {return min_x_;}
float Cell::GetMaxX() const {return max_x_;}
float Cell::GetMinY() const {return min_y_;}
float Cell::GetMaxY() const {return max_y_;}
bool Cell::GetIsLeftExplored() const {return is_left_explored_;}
bool Cell::GetIsRightExplored() const {return is_right_explored_;}
bool Cell::GetIsCeilingExplored() const {return is_ceiling_explored_;}
bool Cell::GetIsFloorExplored() const {return is_floor_explored_;}
bool Cell::GetCompletelyExplored() const {return completely_explored_;}
// need to return a reference of vector
// so that the vector is modifiable outside
std::vector<std::tuple<float2,float2> > & Cell::GetPlaceholder() {
  return placeholder_;
}

void Cell::SetMinX(const float min_x) {min_x_=min_x;}
void Cell::SetMaxX(const float max_x) {max_x_=max_x;}
void Cell::SetMinY(const float min_y) {min_y_=min_y;}
void Cell::SetMaxY(const float max_y) {max_y_=max_y;}
void Cell::SetIsLeftExplored(const bool is_left_explored) {
  is_left_explored_=is_left_explored;
}
void Cell::SetIsRightExplored(const bool is_right_explored) {
  is_right_explored_=is_right_explored_;
}
void Cell::SetIsCeilingExplored(const bool is_ceiling_explored) {
  is_ceiling_explored_=is_ceiling_explored;
}
void Cell::SetIsFloorExplored(const bool is_floor_explored) {
  is_floor_explored_=is_floor_explored;
}
void Cell::SetCompletelyExplored(const bool completely_explored) {
  completely_explored_=completely_explored;
}

std::string Cell::ToString() const {
  std::ostringstream stream;
  stream<<"[Cell] x in ["<<min_x_<<", "<<max_x_
    <<"]; y in ["<<min_y_<<", "<<max_y_<<"]\n";
  stream<<"is_explored=(left="<<is_left_explored_
    <<"); (right="<<is_right_explored_
    <<"); (ceiling="<<is_ceiling_explored_
    <<"); (floor="<<is_floor_explored_
    <<")\n";
  stream<<"placeholder=";
  for (uint i = 0; i < placeholder_.size(); i++) {
    stream<<"[("<<std::get<0>(placeholder_[i]).x
      <<", "<<std::get<0>(placeholder_[i]).y
      <<")=>("<<std::get<1>(placeholder_[i]).x
      <<", "<<std::get<1>(placeholder_[i]).y
      <<")], ";
  }
  stream<<"\n";
  stream<<"is_completely_explored="<<completely_explored_<<"\n";
  return stream.str();
}
