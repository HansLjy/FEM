#include "FileIO.hpp"

class ObjIO : public FileIO {
public:
	void LoadFromFile(const std::string &filename, VectorXd &vertices, MatrixXi &topo) const override;
	void SaveToFile(const std::string &filename, const VectorXd &vertices, const MatrixXi &topo) const override;
};