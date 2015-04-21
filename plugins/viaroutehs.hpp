/*

Copyright (c) 2015, Project OSRM contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*
RFPV:

1 - route_parameters.hpp - tiene la info de la ruta. Allí estan las coordenadas pasadas.

2 - internal_datafacade.hpp lee los datos y tiene varias funciones que son de utilidad.
    EdgeID FindEdge(const NodeID from, const NodeID to)
    EdgeID FindEdgeInEitherDirection(const NodeID from, const NodeID to)
    FixedPointCoordinate GetCoordinateOfNode(const unsigned id)
    EdgeDataT &GetEdgeData(const EdgeID e)
    std::string get_name_for_id(const unsigned name_id)
    unsigned GetNumberOfNodes()
    unsigned GetNumberOfEdges()

3 - phantom_node.hpp define los nodos fantasmas: punto sobre los ejes (puede haber mas de uno) mas cercanos a un punto de ruta especificado por el usuario.
    NodeID forward_node_id;
    NodeID reverse_node_id;
    FixedPointCoordinate location;
    unsigned name_id;
    unsigned component_id;
    unsigned short fwd_segment_position;
    bool is_valid() const;
    bool is_bidirected() const;
    bool is_in_tiny_component() const;

4 -




1 - P1 variar levemente el inicio en la misma cuadra

    N4                N5                 N6
    +------Muller-----+------Muller-------+
    |                 |                   |
  Frugoni             |                   |
    |              Acevedo                |
 P4 *                 |                Jackson
    |                 |                   |
    | P2              |  P3    P1         |
    +--*----Piera-----+---*----*--Piera---+
    N3                N2                  N1

N1 = 45224
N2 = 29263
N3 =
N4 =


Dos puntos en Piera (doble via) -> P1 y P2
http://localhost:5000/viaroutehs?loc=-34.913596,-56.173020&loc=-34.913763,-56.174412&geometry=true
[info] ** Phanton node 1 info **
[info] node1: 29623, node2: 45224, name: 4268, fwd-w: 34, rev-w: 44, fwd-o: 0, rev-o: 0, geom: 4294967295, comp: 0, pos: 0, loc: (-34.9137,-56.173)
[info] En calle: Doctor Luis Piera
[info] forward_node_id: (-34.8199,-56.2061)
[info] reverse_node_id: (-34.864,-56.1339)
[info] ** Phanton node 1 info **
[info] node1: 12141, node2: 45223, name: 4268, fwd-w: 15, rev-w: 53, fwd-o: 0, rev-o: 0, geom: 4294967295, comp: 0, pos: 0, loc: (-34.9138,-56.1744)
[info] En calle: Doctor Luis Piera
[info] forward_node_id: (-34.8594,-56.1336)
[info] reverse_node_id: (-34.8792,-56.1245)

Dos puntos en Piera (doble via) el primero se varia levemente  -> P3 y P2
http://localhost:5000/viaroutehs?loc=-34.913690,-56.173323&loc=-34.913763,-56.174412&geometry=true
[info] ** Phanton node 1 info **
[info] node1: 29623, node2: 45224, name: 4268, fwd-w: 56, rev-w: 22, fwd-o: 0, rev-o: 0, geom: 4294967295, comp: 0, pos: 0, loc: (-34.9137,-56.1733)
[info] En calle: Doctor Luis Piera
[info] forward_node_id: (-34.8199,-56.2061)
[info] reverse_node_id: (-34.864,-56.1339)
[info] ** Phanton node 1 info **
[info] node1: 12141, node2: 45223, name: 4268, fwd-w: 15, rev-w: 53, fwd-o: 0, rev-o: 0, geom: 4294967295, comp: 0, pos: 0, loc: (-34.9138,-56.1744)
[info] En calle: Doctor Luis Piera
[info] forward_node_id: (-34.8594,-56.1336)
[info] reverse_node_id: (-34.8792,-56.1245)

Un punto en Piera (doble via) otro en Frugoni justo despues del anterior -> P1 a P4
http://localhost:5000/viaroutehs?loc=-34.913690,-56.173323&loc=-34.913538,-56.174544&geometry=true
[info] ** Phanton node 1 info **
[info] node1: 29623, node2: 45224, name: 4268, fwd-w: 56, rev-w: 22, fwd-o: 0, rev-o: 0, geom: 4294967295, comp: 0, pos: 0, loc: (-34.9137,-56.1733)
[info] En calle: Doctor Luis Piera
[info] forward_node_id: (-34.8199,-56.2061)
[info] reverse_node_id: (-34.864,-56.1339)
[info] ** Phanton node 1 info **
[info] node1: 12140, node2: 4294967295, name: 2907, fwd-w: 23, rev-w: 77, fwd-o: 0, rev-o: 0, geom: 4294967295, comp: 0, pos: 0, loc: (-34.9135,-56.1747)
[info] En calle: Doctor Emilio Frugoni
[info] forward_node_id: (-34.8672,-56.1092)
[info] reverse_node_id no es valido


*/



#ifndef VIA_ROUTE_HS_HPP
#define VIA_ROUTE_HS_HPP

#include "plugin_base.hpp"

#include "../algorithms/object_encoder.hpp"
#include "../data_structures/search_engine.hpp"
#include "../descriptors/descriptor_base.hpp"
#include "../descriptors/gpx_descriptor.hpp"
#include "../descriptors/json_descriptor.hpp"
#include "../util/integer_range.hpp"
#include "../util/json_renderer.hpp"
#include "../util/make_unique.hpp"
#include "../util/simple_logger.hpp"

#include <osrm/json_container.hpp>

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

template <class DataFacadeT> class ViaRoutePluginHS final : public BasePlugin
{
  private:
    DescriptorTable descriptor_table;                                   //
    std::string descriptor_string;                                      //
    std::unique_ptr<SearchEngine<DataFacadeT>> search_engine_ptr;       // El motor de busqueda
    DataFacadeT *facade;                                                // Los datos

  public:
    explicit ViaRoutePluginHS(DataFacadeT *facade) : descriptor_string("viaroutehs"), facade(facade)
    {
        search_engine_ptr = osrm::make_unique<SearchEngine<DataFacadeT>>(facade);

        descriptor_table.emplace("json", 0);
        descriptor_table.emplace("gpx", 1);
        // descriptor_table.emplace("geojson", 2);
    }

    virtual ~ViaRoutePluginHS() {}

    const std::string GetDescriptor() const override final { return descriptor_string; }

    int HandleRequest(const RouteParameters &route_parameters,
                      osrm::json::Object &json_result) override final
    {
        if (!check_all_coordinates(route_parameters.coordinates))
        {
            return 400;
        }

        // Info general
        SimpleLogger().Write(logINFO) << "**************************************";
        unsigned int nn = facade->GetNumberOfNodes();
        unsigned int ne = facade->GetNumberOfEdges();
        SimpleLogger().Write(logINFO) << "timestamp: "<< facade->GetTimestamp();
        SimpleLogger().Write(logINFO) << "son "<< nn << " nodes y " << ne << " edges;";

        // Creo un vector de pares de nodos fantasmas del mismo tamaño de coordenadas de la ruta
        // phantom_node_pair ---> std::pair<PhantomNode, PhantomNode>
        std::vector<phantom_node_pair> phantom_node_pair_list(route_parameters.coordinates.size());
        const bool checksum_OK = (route_parameters.check_sum == facade->GetCheckSum());

        for (const auto i : osrm::irange<std::size_t>(0, route_parameters.coordinates.size()))
        {
            if (checksum_OK && i < route_parameters.hints.size() &&
                !route_parameters.hints[i].empty())
            {
                ObjectEncoder::DecodeFromBase64(route_parameters.hints[i],
                                                phantom_node_pair_list[i]);
                if (phantom_node_pair_list[i].first.is_valid(facade->GetNumberOfNodes()))
                {
                    continue;
                }
            }
            std::vector<PhantomNode> phantom_node_vector;
            if (facade->IncrementalFindPhantomNodeForCoordinate(route_parameters.coordinates[i],
                                                                phantom_node_vector, 1))
            {
                BOOST_ASSERT(!phantom_node_vector.empty());
                phantom_node_pair_list[i].first = phantom_node_vector.front();
                if (phantom_node_vector.size() > 1)
                {
                    phantom_node_pair_list[i].second = phantom_node_vector.back();
                }

                /* */
                // Infos
                SimpleLogger().Write(logINFO) << "** Phanton node 1 info **";
                SimpleLogger().Write(logINFO) << phantom_node_pair_list[i].first;
                SimpleLogger().Write(logINFO)
                    << "valid: " <<  phantom_node_pair_list[i].first.is_valid()
                    << " bi_directed: " <<  phantom_node_pair_list[i].first.is_bidirected()
                    << " in_tiny_component: " <<  phantom_node_pair_list[i].first.is_in_tiny_component();
                SimpleLogger().Write(logINFO) << "En calle: " << facade->get_name_for_id(phantom_node_pair_list[i].first.name_id);
                SimpleLogger().Write(logINFO) << "El NodeID es: " << facade->get_node_id_for_id(phantom_node_pair_list[i].first.forward_node_id);

                FixedPointCoordinate result;
                facade->LocateClosestEndPointForCoordinate(phantom_node_pair_list[i].first.location,
                                                        result);
                SimpleLogger().Write(logINFO) << "Punto mas cercano es: " << result;


                //SimpleLogger().Write(logINFO) << edge.first;
                //SimpleLogger().Write(logINFO) << edge.second;

                if ( phantom_node_pair_list[i].first.forward_node_id <= nn ) {
                    SimpleLogger().Write(logINFO) << "GetCoordinateOfNode de forward_node_id: " << facade->GetCoordinateOfNode(phantom_node_pair_list[i].first.forward_node_id);
                    // Los edeges ID
                    EdgeID be = facade->BeginEdges(phantom_node_pair_list[i].first.forward_node_id);
                    SimpleLogger().Write(logINFO) << "--- Begin ---";
                    SimpleLogger().Write(logINFO) << "BeginEdges(" << phantom_node_pair_list[i].first.forward_node_id << ")="<< be;
                    SimpleLogger().Write(logINFO) << "GetTarget(" << be << ")=" << facade->GetTarget(be);
                    SimpleLogger().Write(logINFO) << "GetCoordinateOfNode(" << facade->GetTarget(be) << ")=" <<
                        facade->GetCoordinateOfNode( facade->GetTarget(be) );

                    EdgeID ee = facade->EndEdges(phantom_node_pair_list[i].first.forward_node_id);
                    SimpleLogger().Write(logINFO) << "--- End ---";
                    SimpleLogger().Write(logINFO) << "EndEdges(" << phantom_node_pair_list[i].first.forward_node_id << ")="<< ee;
                    SimpleLogger().Write(logINFO) << "GetTarget(" << ee << ")=" << facade->GetTarget(ee);
                    SimpleLogger().Write(logINFO) << "GetCoordinateOfNode(" << facade->GetTarget(ee) << ")=" <<
                        facade->GetCoordinateOfNode( facade->GetTarget(ee) );

                    // Los datos de los EDGES
                    using EdgeData = typename DataFacadeT::EdgeData;
                    const EdgeData &bedge = facade->GetEdgeData(be);
                    const EdgeData &eedge = facade->GetEdgeData(ee);
                    SimpleLogger().Write(logINFO) << "--- EdgeData ---";
                    SimpleLogger().Write(logINFO) << "Begin EdgeData (" << be << "): " <<
                        bedge.id << " | " << bedge.distance << " | " <<
                        bedge.forward << " | " << bedge.backward;

                    SimpleLogger().Write(logINFO) << "End EdgeData (" << ee << "): " <<
                        eedge.id << " | " << eedge.distance << " | " <<
                        eedge.forward << " | " << eedge.backward;
                    /*
                    SimpleLogger().Write(logINFO) << "GetCoordinateOfNode de id: " << facade->GetCoordinateOfNode(
                      facade->get_node_id_for_id(phantom_node_pair_list[i].first.forward_node_id)
                    );
                    */
                } else {
                    SimpleLogger().Write(logINFO) << "forward_node_id no es valido";
                }
                if ( phantom_node_pair_list[i].first.reverse_node_id <= nn ) {
                    SimpleLogger().Write(logINFO) << "reverse_node_id: " << facade->GetCoordinateOfNode(phantom_node_pair_list[i].first.reverse_node_id);
                } else {
                    SimpleLogger().Write(logINFO) << "reverse_node_id no es valido";
                }

                if (phantom_node_vector.size() > 1)
                {
                    SimpleLogger().Write(logINFO) << "** Phanton node 2 info **";
                    SimpleLogger().Write(logINFO) << phantom_node_pair_list[i].second;
                }
                /* */

            }
        }

        auto check_component_id_is_tiny = [](const phantom_node_pair &phantom_pair)
        {
            return phantom_pair.first.component_id != 0;
        };

        const bool every_phantom_is_in_tiny_cc =
            std::all_of(std::begin(phantom_node_pair_list), std::end(phantom_node_pair_list),
                        check_component_id_is_tiny);

        // are all phantoms from a tiny cc?
        const auto component_id = phantom_node_pair_list.front().first.component_id;

        auto check_component_id_is_equal = [component_id](const phantom_node_pair &phantom_pair)
        {
            return component_id == phantom_pair.first.component_id;
        };

        const bool every_phantom_has_equal_id =
            std::all_of(std::begin(phantom_node_pair_list), std::end(phantom_node_pair_list),
                        check_component_id_is_equal);

        auto swap_phantom_from_big_cc_into_front = [](phantom_node_pair &phantom_pair)
        {
            if (0 != phantom_pair.first.component_id)
            {
                std::swap(phantom_pair.first, phantom_pair.second);
            }
        };

        // this case is true if we take phantoms from the big CC
        if (!every_phantom_is_in_tiny_cc || !every_phantom_has_equal_id)
        {
            std::for_each(std::begin(phantom_node_pair_list), std::end(phantom_node_pair_list),
                          swap_phantom_from_big_cc_into_front);
        }

        InternalRouteResult raw_route;
        auto build_phantom_pairs =
            [&raw_route](const phantom_node_pair &first_pair, const phantom_node_pair &second_pair)
        {
            raw_route.segment_end_coordinates.emplace_back(
                PhantomNodes{first_pair.first, second_pair.first});
        };
        osrm::for_each_pair(phantom_node_pair_list, build_phantom_pairs);

        if (route_parameters.alternate_route && 1 == raw_route.segment_end_coordinates.size())
        {
            search_engine_ptr->alternative_path(raw_route.segment_end_coordinates.front(),
                                                raw_route);
        }
        else
        {
            search_engine_ptr->shortest_path(raw_route.segment_end_coordinates,
                                             route_parameters.uturns, raw_route);
        }

        if (INVALID_EDGE_WEIGHT == raw_route.shortest_path_length)
        {
            SimpleLogger().Write(logDEBUG) << "Error occurred, single path not found";
        }

        std::unique_ptr<BaseDescriptor<DataFacadeT>> descriptor;
        switch (descriptor_table.get_id(route_parameters.output_format))
        {
        case 1:
            descriptor = osrm::make_unique<GPXDescriptor<DataFacadeT>>(facade);
            break;
        // case 2:
        //      descriptor = osrm::make_unique<GEOJSONDescriptor<DataFacadeT>>();
        //      break;
        default:
            descriptor = osrm::make_unique<JSONDescriptor<DataFacadeT>>(facade);
            break;
        }

        descriptor->SetConfig(route_parameters);
        descriptor->Run(raw_route, json_result);
        return 200;
    }
};

#endif // VIA_ROUTE_HS_HPP
