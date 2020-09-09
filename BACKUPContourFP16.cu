#include <vtkm/cont/DataSet.h>
#include <vtkm/cont/Initialize.h>
#include <vtkm/io/reader/VTKDataSetReader.h>
#include <vtkm/io/writer/VTKDataSetWriter.h>
#include <vtkm/cont/DataSetBuilderUniform.h>
#include <vtkm/cont/DataSetFieldAdd.h>
#include <vtkm/Types.h>
#include <vtkm/cont/ArrayHandle.h>
#include <vtkm/cont/Logging.h>
#include <vtkm/cont/Timer.h>
#include <vtkm/filter/FilterDataSetWithField.h>
#include <vtkm/worklet/Contour.h>
#include <vtkm/worklet/SurfaceNormals.h>
#include <vtkm/cont/VariantArrayHandle.h>
#include <vtkm/rendering/Actor.h>
#include <vtkm/rendering/CanvasRayTracer.h>
#include <vtkm/rendering/MapperRayTracer.h>
#include <vtkm/rendering/MapperWireframer.h>
#include <vtkm/rendering/Scene.h>
#include <vtkm/rendering/View3D.h>
#include <vtkm/cont/ColorTable.h>
#include <iostream>
#include <stdio.h>
#include <string>

namespace vtkm
{

namespace worklet
{
	class ContourFP16
	{
		public:
               //----------------------------------------------------------------------------
               ContourFP16(bool mergeDuplicates = true): SharedState(mergeDuplicates){}

               //----------------------------------------------------------------------------
               vtkm::cont::ArrayHandle<vtkm::Id2> GetInterpolationEdgeIds() const{return this->SharedState.InterpolationEdgeIds;}

                //----------------------------------------------------------------------------
                void SetMergeDuplicatePoints(bool merge) { this->SharedState.MergeDuplicatePoints = merge; }

                //----------------------------------------------------------------------------
                bool GetMergeDuplicatePoints() const { return this->SharedState.MergeDuplicatePoints; }

                //----------------------------------------------------------------------------
                vtkm::cont::ArrayHandle<vtkm::Id> GetCellIdMap() const { return this->SharedState.CellIdMap; }

		//----------------------------------------------------------------------------
  		void ReleaseCellMapArrays() { this->SharedState.CellIdMap.ReleaseResources(); }

		vtkm::cont::CellSetSingleType<> Run(
				const std::vector<vtkm::Float32>& isovalues,
				const const vtkm::cont::DynamicCellSet&& cells,
				const vtkm::cont::detail::CoordDataDepWrapper& coordinateSystem,
				const vtkm::cont::ArrayHandle<vtkm::Float16, vtkm::cont::StorageTagBasic>& input,
				vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Float16, 3>, vtkm::cont::StorageTagBasic>& vertices)
		{
			this->SharedState.GenerateNormals = false;
    			vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Float16, 3>> normals;
			vtkm::cont::CellSetSingleType<> outputCells;
    			vtkm::cont::CastAndCall(cells,
                            contour::DeduceCellType{},
                            coordinateSystem,
                            outputCells,
                            isovalues,
                            input,
                            vertices,
                            normals,
                            this->SharedState);
    			return outputCells;
		}
		
		private:
 	 	vtkm::worklet::contour::CommonState SharedState;
	};	
}


namespace filter
{
struct PolicyFP16DataSet : vtkm::filter::PolicyBase<PolicyFP16DataSet>
{
public:
  struct TypeListTagFP16 : vtkm::ListTagBase<::vtkm::UInt8,
                                             ::vtkm::Int32,
                                             ::vtkm::Int64,
                                             ::vtkm::Float16,
                                             ::vtkm::Float32,
                                             ::vtkm::Float64,
                                             ::vtkm::Vec<::vtkm::Float16, 3>,
                                             ::vtkm::Vec<::vtkm::Float32, 3>,
                                             ::vtkm::Vec<::vtkm::Float64, 3>>
  {
  };

  using FieldTypeList = TypeListTagFP16;
};

namespace
{

template <typename CellSetList>
inline bool IsCellSetStructured(const vtkm::cont::DynamicCellSetBase<CellSetList>& cellset)
{
  if (cellset.template IsType<vtkm::cont::CellSetStructured<1>>() ||
      cellset.template IsType<vtkm::cont::CellSetStructured<2>>() ||
      cellset.template IsType<vtkm::cont::CellSetStructured<3>>())
  {
    return true;
  }
  return false;
}
} // anonymous namespace

class ContourFP16 : public vtkm::filter::FilterDataSetWithField<ContourFP16>
{
	public :
  	ContourFP16();
  	void SetNumberOfIsoValues(vtkm::Id num);
	vtkm::Id GetNumberOfIsoValues() const;
	void SetIsoValue(vtkm::Float32 v) { this->SetIsoValue(0, v); }
	void SetIsoValue(vtkm::Id index, vtkm::Float32);
	void SetIsoValues(const std::vector<vtkm::Float32>& values);
	vtkm::Float32 GetIsoValue(vtkm::Id index) const;

	VTKM_CONT
  	void SetMergeDuplicatePoints(bool on) { this->Worklet.SetMergeDuplicatePoints(on); }
  	VTKM_CONT
  	bool GetMergeDuplicatePoints() const { return this->Worklet.GetMergeDuplicatePoints(); }

	VTKM_CONT
  	void SetGenerateNormals(bool on) { this->GenerateNormals = on; }
  	VTKM_CONT
  	bool GetGenerateNormals() const { return this->GenerateNormals; }

	 VTKM_CONT
  	void SetNormalArrayName(const std::string& name) { this->NormalArrayName = name; }
  	VTKM_CONT
  	const std::string& GetNormalArrayName() const { return this->NormalArrayName; }



	//template <typename T, typename S, typename Policy>
	VTKM_CONT cont::DataSet ContourFP16::DoExecute(
  	const vtkm::cont::DataSet& input,
 	const vtkm::cont::ArrayHandle<vtkm::Float16, vtkm::cont::StorageTagBasic>& field,
  	const vtkm::filter::FieldMetadata& fieldMeta,
  	filter::PolicyBase<PolicyFP16DataSet> policy)
	{
		if (fieldMeta.IsPointField() == false)
  		{
    			throw vtkm::cont::ErrorFilterExecution("Point field expected.");
  		}

  		if (this->IsoValues.size() == 0)
  		{
    			throw vtkm::cont::ErrorFilterExecution("No iso-values provided.");
  		}

		const vtkm::Id numFields = input.GetNumberOfFields();
  		bool hasCellFields = false;
  		for (vtkm::Id fieldIdx = 0; fieldIdx < numFields && !hasCellFields; ++fieldIdx)
  		{
    			auto f = input.GetField(fieldIdx);
    			hasCellFields = f.IsFieldCell();
  		}

		 //get the cells and coordinates of the dataset
  		const vtkm::cont::DynamicCellSet& cells = input.GetCellSet();

  		const vtkm::cont::CoordinateSystem& coords = input.GetCoordinateSystem
			(this->GetActiveCoordinateSystemIndex());

  		using Vec3HandleType = vtkm::cont::ArrayHandle<vtkm::Vec3f_16>;
  		Vec3HandleType vertices;
  		Vec3HandleType normals;

  		vtkm::cont::DataSet output;
  		vtkm::cont::CellSetSingleType<> outputCells;

  		std::vector<vtkm::Float32> ivalues(this->IsoValues.size());
  		for (std::size_t i = 0; i < ivalues.size(); ++i)
  		{
    			ivalues[i] = static_cast<vtkm::Float32>(this->IsoValues[i]);
  		}

		bool generateHighQualityNormals = IsCellSetStructured(cells)
			? !this->ComputeFastNormalsForStructured
			: !this->ComputeFastNormalsForUnstructured;
	/*	if (this->GenerateNormals && generateHighQualityNormals)
		{
    			outputCells = this->Worklet.Run(ivalues,
                                    vtkm::filter::ApplyPolicyCellSet(cells, policy, *this),
                                    coords.GetData(),
                                    field,
                                    vertices,
                                    normals);
  		}
  		else
  		{
    	*/		outputCells = this->Worklet.Run(ivalues,
                                    vtkm::filter::ApplyPolicyCellSet(cells, policy, *this),
                                    coords.GetData(),
                                    field,
                                    vertices);
  	//	}
	
		if (this->GenerateNormals)
  		{
    			if (!generateHighQualityNormals)
    			{
      				Vec3HandleType faceNormals;
      				vtkm::worklet::FacetedSurfaceNormals faceted;
      				faceted.Run(outputCells, vertices, faceNormals);
      				vtkm::worklet::SmoothSurfaceNormals smooth;
      				smooth.Run(outputCells, faceNormals, normals);
    			}

    			output.AddField(vtkm::cont::make_FieldPoint(this->NormalArrayName, normals));
  		}

  		if (this->AddInterpolationEdgeIds)
  		{
    			vtkm::cont::Field interpolationEdgeIdsField(InterpolationEdgeIdsArrayName,
                                                vtkm::cont::Field::Association::POINTS,
                                                this->Worklet.GetInterpolationEdgeIds());
    			output.AddField(interpolationEdgeIdsField);
  		}

  		//assign the connectivity to the cell set
  		output.SetCellSet(outputCells);

  		//add the coordinates to the output dataset
  		vtkm::cont::CoordinateSystem outputCoords("coordinates", vertices);
  		output.AddCoordinateSystem(outputCoords);

  	/*	if (!hasCellFields)
  		{
    			this->Worklet.ReleaseCellMapArrays();
  		}
*/
		 return output;
	}


	private:
  	std::vector<vtkm::Float32> IsoValues;
  	bool GenerateNormals;
  	std::string NormalArrayName;
  	bool AddInterpolationEdgeIds;
  	bool ComputeFastNormalsForStructured;
  	bool ComputeFastNormalsForUnstructured;
  	std::string InterpolationEdgeIdsArrayName;
  	vtkm::worklet::ContourFP16 Worklet;
};

template<>
  struct FilterTraits<vtkm::filter::ContourFP16>
  {
    using InputFieldTypeList = vtkm::ListTagBase<vtkm::Float16>;
  };

}
}

int main(int argc, char** argv)
{
	auto opts = vtkm::cont::InitializeOptions::DefaultAnyDevice;

	// SetLogLevelName must be called before Initialize
	vtkm::cont::SetLogLevelName(vtkm::cont::LogLevel::Info, "flyingedges_log");
	vtkm::cont::SetStderrLogLevel(vtkm::cont::LogLevel::Info);
	vtkm::cont::InitializeResult config = vtkm::cont::Initialize(argc, argv, opts);

	vtkm::cont::DataSet dataSet;
	vtkm::cont::DataSetBuilderUniform dataSetBuilder;
	vtkm::cont::DataSetFieldAdd dataSetFieldAdd;
	
	vtkm::Id3 dims(3, 3, 3);
  	vtkm::Id3 org(0,0,0);
  	vtkm::Id3 spc(1,1,1);
	dataSet = dataSetBuilder.Create(dims, org, spc);

	vtkm::Int64 N = 27;
	std::vector<vtkm::Float16> data(N);
  	for (vtkm::Int64 i = 0; i < N; i++)
    		data[i] = (float)i;
  	vtkm::cont::ArrayHandle<vtkm::Float16> fieldData = vtkm::cont::make_ArrayHandle(data);
	std::string fieldName = "test_field";
  	dataSetFieldAdd.AddPointField(dataSet, fieldName, fieldData);

	/*vtkm::Id3 dims(150 , 150 , 150);
  	vtkm::Id3 org(0,0,0);
  	vtkm::Id3 spc(1,1,1);
	 dataSet = dataSetBuilder.Create(dims, org, spc);

	std::vector<float> data_u(257*257*257*sizeof(float));
	std::vector<float> data_v(257*257*257*sizeof(float));
	//std::vector<vtkm::Float16> pointvar_u(257*257*257*sizeof(vtkm::Float16));
        //std::vector<vtkm::Float16> pointvar_v(257*257*257*sizeof(vtkm::Float16));

	float* array =(float*)malloc(257*257*257*sizeof(float));
	FILE *ff1 = fopen("data/u-1.bin", "r");
	fread(array, 257*257*257*sizeof(float), 1, ff1);
	fclose(ff1);
	for(long long i=0; i<(long long)257*257*257; ++i)
	{
		data_u[i] = (float)array[i];
		//vtkm::Float16 th_data(array[i]);
		//pointvar_u.push_back(th_data.get());
	}
      
	FILE *ff2 = fopen("data/v-1.bin", "r");
	fread(array, 257*257*257*sizeof(float), 1, ff2);
	fclose(ff2);
	for(long long j=0; j<(long long)257*257*257; ++j)
	{
		//vtkm::Float16 th_data(array[j]);
		//pointvar_v.push_back(th_data.get());
		data_v[j] = (float)array[j];
	}
	
	vtkm::cont::ArrayHandle<vtkm::Float16> pointvar_u = vtkm::cont::make_ArrayHandle(data_u);
        vtkm::cont::ArrayHandle<vtkm::Float16> pointvar_v = vtkm::cont::make_ArrayHandle(data_v);
	std::string dfname1 = "pointvar_u";
	std::string dfname2 = "pointvar_v";
	
	//dataSetFieldAdd.AddPointField(dataSet, dfname1, data_u);
	dataSetFieldAdd.AddPointField(dataSet, dfname1, pointvar_u);
	dataSetFieldAdd.AddPointField(dataSet, dfname2, pointvar_v);
	*/
	
	vtkm::filter::ContourFP16 contour;
	contour.SetGenerateNormals(false);
	contour.SetMergeDuplicatePoints(false);
	contour.SetNumberOfIsoValues(1);
	contour.SetIsoValue(0, 0.5);
	contour.SetActiveField(fieldName);
	//contour.SetFieldsToPass({dfname1,dfname2});
	
	vtkm::cont::DataSet ds_from_mc = contour.Execute(dataSet,vtkm::filter::PolicyFP16DataSet());
	/*vtkm::io::writer::VTKDataSetWriter writer("out_mc.vtk");
	writer.WriteDataSet(ds_from_mc);
*/
	return 0;
}
