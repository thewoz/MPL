/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2017
 * Created by Leonardo Parisi (leonardo.parisi[at]gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef _H_MPL_BIMAP_H_
#define _H_MPL_BIMAP_H_

#include <unordered_map>
#include <optional>

//****************************************************************************/
// namespace std
//****************************************************************************/
namespace std {

  //****************************************************************************/
  // class bimap
  //****************************************************************************/
  template <class Left, class Right>
  class bimap {
    
  private:
    
    std::unordered_map<Left,  Right> left_to_right;
    std::unordered_map<Right, Left>  right_to_left;
    
  public:
    
    //****************************************************************************/
    // insert()
    //****************************************************************************/
    void insert(const Left & l, const Right & r) {
      erase_left(l);
      erase_right(r);
      left_to_right.emplace(l, r);
      right_to_left.emplace(r, l);
    }
    
    //****************************************************************************/
    // find_right()
    //****************************************************************************/
    std::optional<Right> find_right(const Left & l) const {
      if(auto it = left_to_right.find(l); it != left_to_right.end()) return it->second;
      return std::nullopt;
    }
    
    //****************************************************************************/
    // find_left()
    //****************************************************************************/
    std::optional<Left> find_left(const Right & r) const {
      if(auto it = right_to_left.find(r); it != right_to_left.end()) return it->second;
      return std::nullopt;
    }
    
    //****************************************************************************/
    // contains()
    //****************************************************************************/
    bool contains_left(const Left & l) const  { return left_to_right.count(l) != 0; }
    bool contains_right(const Right & r) const{ return right_to_left.count(r) != 0; }
    
    //****************************************************************************/
    // erase_left()
    //****************************************************************************/
    bool erase_left(const Left & l) {
      auto it = left_to_right.find(l);
      if(it == left_to_right.end()) return false;
      right_to_left.erase(it->second);
      left_to_right.erase(it);
      return true;
    }
    
    //****************************************************************************/
    // erase_right()
    //****************************************************************************/
    bool erase_right(const Right & r) {
      auto it = right_to_left.find(r);
      if(it == right_to_left.end()) return false;
      left_to_right.erase(it->second);
      right_to_left.erase(it);
      return true;
    }
    
    //****************************************************************************/
    // clear()
    //****************************************************************************/
    void clear() { left_to_right.clear(); right_to_left.clear(); }
    
    //****************************************************************************/
    // size()
    //****************************************************************************/
    std::size_t size() const { return left_to_right.size(); }
    
    //****************************************************************************/
    // empty()
    //****************************************************************************/
    bool empty() const { return left_to_right.empty(); }
    
    //****************************************************************************/
    //
    //****************************************************************************/
    const std::unordered_map<Left, Right> & left()  const { return left_to_right; }
    const std::unordered_map<Right, Left> & right() const { return right_to_left; }
    
    //****************************************************************************/
    //
    //****************************************************************************/
    const Right & left(const Left & l)   const { return left_to_right.at(l); }
    const Left  & right(const Right & r) const { return right_to_left.at(r); }
    
    //****************************************************************************/
    // iterations
    //****************************************************************************/
    using left_const_iterator  = typename std::unordered_map<Left,  Right>::const_iterator;
    using right_const_iterator = typename std::unordered_map<Right, Left>::const_iterator;
    
    left_const_iterator  left_begin()  const { return left_to_right.cbegin(); }
    left_const_iterator  left_end()    const { return left_to_right.cend();   }
    right_const_iterator right_begin() const { return right_to_left.cbegin(); }
    right_const_iterator right_end()   const { return right_to_left.cend();   }
    
  };

} // namespace std

#endif
