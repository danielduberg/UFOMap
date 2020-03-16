#ifndef UFOMAP_MAPPING_SERVER_NODELET_H
#define UFOMAP_MAPPING_SERVER_NODELET_H

#include <nodelet/nodelet.h>

#include <ufomap_mapping/server.h>

#include <memory>

namespace ufomap_mapping
{
class UFOMapServerNodelet : public nodelet::Nodelet
{
private:
	std::shared_ptr<UFOMapServer> server_;

public:
	void onInit() override;
};
}  // namespace ufomap_mapping

#endif  // UFOMAP_MAPPING_SERVER_NODELET_H
