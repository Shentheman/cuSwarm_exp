#ifndef CELL_H_
#define CELL_H_

#include <sstream>
#include <vector>
#include <tuple>
#include "kernels.cuh"

class Cell {
public:
  Cell();
  Cell(float min_x, float max_x, float min_y, float max_y, 
      bool is_left_explored=true, bool is_right_explored=true, 
      bool is_ceiling_explored=true, bool is_floor_explored=true);

  float GetMinX() const;
  float GetMaxX() const;
  float GetMinY() const;
  float GetMaxY() const;
  bool GetIsLeftExplored() const;
  bool GetIsRightExplored() const;
  bool GetIsCeilingExplored() const;
  bool GetIsFloorExplored() const;
  bool GetCompletelyExplored() const;
  std::vector<std::tuple<float2,float2> > & GetPlaceholder();

  void SetMinX(const float min_x);
  void SetMaxX(const float max_x);
  void SetMinY(const float min_y);
  void SetMaxY(const float max_y);
  void SetIsLeftExplored(const bool is_left_explored);
  void SetIsRightExplored(const bool is_right_explored);
  void SetIsCeilingExplored(const bool is_ceiling_explored);
  void SetIsFloorExplored(const bool is_floor_explored);
  void SetCompletelyExplored(const bool completely_explored);

  std::string ToString() const;

  private:
    float min_x_;
    float max_x_;
    float min_y_;
    float max_y_;
    bool is_left_explored_;
    bool is_right_explored_;
    bool is_ceiling_explored_;
    bool is_floor_explored_;
    std::vector<std::tuple<float2,float2> > placeholder_;
    bool completely_explored_;
  };

#endif
