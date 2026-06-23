/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2017-2026
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

#ifndef _H_MPL_CONTAINER_GRAPH_H_
#define _H_MPL_CONTAINER_GRAPH_H_

#include <cstdio>
#include <cstdlib>

#include <vector>
#include <string>
#include <map>

//****************************************************************************/
// namespace mpl
//****************************************************************************/
namespace mpl {

  //****************************************************************************/
  // namespace graph
  //****************************************************************************/
  namespace graph {

    //****************************************************************************/
    // class node_t
    //****************************************************************************/
    class node_t {

    public:
      
      int frame;
            
      std::vector<int> parents;
      std::vector<int> children;
      
      node_t() : frame(-1) { }

      node_t(int _frame) : frame(_frame) { }

      //****************************************************************************/
      //  isRoot
      //****************************************************************************/
      inline bool isRoot() const { return sizeParents() == 0; }

      //****************************************************************************/
      //  isTail
      //****************************************************************************/
      inline bool isTail() const { return sizeChildren() == 0; }
      
      //****************************************************************************/
      //  addParents
      //****************************************************************************/
      inline void addParents(int id) { parents.push_back(id); }
      
      //****************************************************************************/
      //  addChildren
      //****************************************************************************/
      inline void addChildren(int id) { children.push_back(id); }
      
      //****************************************************************************/
      //  size
      //****************************************************************************/
      inline size_t sizeParents()  const { return parents.size();  }
      inline size_t sizeChildren() const { return children.size(); }
      
      //****************************************************************************/
      //  operator []
      //****************************************************************************/
      inline const int & operator [] (std::size_t index) const { return children[index]; }
      inline       int & operator [] (std::size_t index)       { return children[index]; }
      
    };

  } // namespace graph
  

  //****************************************************************************/
  //  graph_t
  //****************************************************************************/
  class graph_t {
    
  protected:
    
    // vettore dei nodi
    std::vector<graph::node_t *> nodes;
    
    std::vector<bool> isGood;

    bool isInited;
    
  public:
    
    //****************************************************************************/
    //  graph_t()
    //****************************************************************************/
    graph_t() : isInited(false) { }
    
    //****************************************************************************/
    //  addNode
    //****************************************************************************/
    void addNode(graph::node_t * node) { nodes.push_back(node); isGood.push_back(false); }
    
    //****************************************************************************/
    //  createTracks
    //****************************************************************************/
    void createTracks(std::vector<std::vector<int>> & tracks) {

      if(!isInited) {
        fprintf(stderr, "mpl::graph_t::createTracks() error: the graph must be initialized before calling createTracks()\n");
        abort();
      }
      
      // pulisco le traccie
      tracks.clear();
      
      // ciclo su tutti i nodi
      for(std::size_t i=0; i<nodes.size(); ++i){

        // se il nodo è buono o non è una radice lo salto
        if(!isGood[i] || !nodes[i]->isRoot()) continue;

        // creo tutte le traiettorie a partire da quel punto
        buildTracksForward(i, std::vector<int>(), tracks);

      }
      
    }
    
    //****************************************************************************/
    //  isToAdd
    //****************************************************************************/
    virtual inline bool isToAdd(const std::vector<int> & track) { return true; }
    
  private:
      
    //****************************************************************************/
    //  buildTracksForward
    //****************************************************************************/
    void buildTracksForward(std::size_t id, std::vector<int> track, std::vector<std::vector<int>> & tracks) {

      // mi segno che sono passato da questo nodo
      isGood[id] = false;

      // aggiungo il nodo alla traccia
      track.push_back((int)id);

      // se il nodo non ha figli il cammino e' finito
      if(nodes[id]->isTail()) {
        if(isToAdd(track)) tracks.push_back(track);
        return;
      }

      // visito tutti i figli del nodo
      for(std::size_t i=0; i<nodes[id]->sizeChildren(); ++i) {
        std::size_t child = (std::size_t)nodes[id]->children[i];
        if(isGood[child])
          buildTracksForward(child, track, tracks);
      }

    }
    
  public:
    
    //****************************************************************************/
    // save
    //****************************************************************************/
    void save(const std::string & filepath) {
      
      if(!isInited) {
        fprintf(stderr, "mpl::graph_t::save() error: the graph must be initialized before calling save()\n");
        abort();
      }
      
      // dot filename.gv -Tpdf -o filename.pdf
      
      FILE * outFile = fopen(filepath.c_str(), "w");
      if(outFile == NULL) {
        fprintf(stderr, "mpl::graph_t::save() error: cannot open the output file\n");
        abort();
      }
      
      fprintf(outFile, "digraph dump {\n");
      
      fprintf(outFile, "\n\n");
      
      std::map<int,std::vector<int>> rank;
      
      for(std::size_t i=0; i<nodes.size(); i++) {

        if(!isGood[i]) continue;

        rank[nodes[i]->frame].push_back((int)i);

        fprintf(outFile, "%zu [label=\"%zu\"];\n", i, i);

      }
      
      fprintf(outFile,"\n\n");
      
      for(auto rankIt=rank.begin(); rankIt!=rank.end(); rankIt++) {
        
        fprintf(outFile,"\t {rank=same;");
        
        for(std::size_t j=0; j<rankIt->second.size(); j++) fprintf(outFile," %d",rankIt->second[j]);
        
        fprintf(outFile,";}\n\n");
        
      }
      
      fprintf(outFile,"\n\n");
      
      for(std::size_t i=0; i<nodes.size(); i++) {
        for(std::size_t j=0; j<nodes[i]->sizeChildren(); ++j) {
          std::size_t child = (std::size_t)nodes[i]->children[j];
          if(isGood[child]) fprintf(outFile, "%zu -> %d \n", i, nodes[i]->children[j]);
        }
      }
      
      fprintf(outFile, "}\n");
      
      fclose(outFile);
      
    }
    
    
  };

} // namespace mpl
  
#endif // _H_MPL_CONTAINER_GRAPH_H_
