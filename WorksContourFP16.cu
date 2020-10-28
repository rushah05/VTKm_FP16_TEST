#include <vtkm/Types.h>
#include <vtkm/cont/DataSet.h>
#include <vtkm/cont/DataSetBuilderUniform.h>
#include <vtkm/cont/Invoker.h>
#include <vtkm/cont/VariantArrayHandle.h>
#include <vtkm/worklet/DispatcherMapField.h>
#include <vtkm/cont/DataSetFieldAdd.h>
#include <vtkm/filter/FilterField.h>
#include <vtkm/filter/FilterDataSetWithField.h>
#include <vtkm/filter/Contour.h>
#include <vtkm/worklet/contour/CommonState.h>
#include <vtkm/cont/ArrayHandle.h>
#include <iostream>
#include <vtkm/filter/CreateResult.h> 
#include <string>
#include <vtkm/cont/Field.h>
namespace vtkm
{
	namespace worklet
	{
		class ContourFP16
		{
			public:
			ContourFP16(bool mergeDuplicates = true)
				: SharedState(mergeDuplicates)
			{
			}

			 //----------------------------------------------------------------------------
			  void SetMergeDuplicatePoints(bool merge) { this->SharedState.MergeDuplicatePoints = merge; }

			  //----------------------------------------------------------------------------
			  bool GetMergeDuplicatePoints() const { return this->SharedState.MergeDuplicatePoints; }
			
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

	 	class ContourFP16 : public vtkm::filter::FilterDataSetWithField<ContourFP16>
        	{
			public:
			//-----------------------------------------------------------------------------
			ContourFP16()
			  : vtkm::filter::FilterDataSetWithField<ContourFP16>()
			  , IsoValues()
			  , GenerateNormals(false)
			  , AddInterpolationEdgeIds(false)
			  , ComputeFastNormalsForStructured(false)
			  , ComputeFastNormalsForUnstructured(true)
			  , NormalArrayName("normals")
			  , InterpolationEdgeIdsArrayName("edgeIds")
			  , Worklet()
			{
			  // todo: keep an instance of marching cubes worklet as a member variable
			}

			//-----------------------------------------------------------------------------
			void SetNumberOfIsoValues(vtkm::Id num)
			{
				if (num >= 0)
				{
					this->IsoValues.resize(static_cast<std::size_t>(num));
				}
			}

			//-----------------------------------------------------------------------------
			vtkm::Id GetNumberOfIsoValues() const
			{
				return static_cast<vtkm::Id>(this->IsoValues.size());
			}

			//-----------------------------------------------------------------------------
			void SetIsoValue(vtkm::Id index, vtkm::Float16 v)
			{
				std::size_t i = static_cast<std::size_t>(index);
				if (i >= this->IsoValues.size())
				{
					this->IsoValues.resize(i + 1);
				}
				this->IsoValues[i] = v;
			}

			//-----------------------------------------------------------------------------
			void SetIsoValues(const std::vector<vtkm::Float16>& values)
			{
				this->IsoValues = values;
			}

			//-----------------------------------------------------------------------------
			vtkm::Float16 GetIsoValue(vtkm::Id index) const
			{
				return this->IsoValues[static_cast<std::size_t>(index)];
			}

			//-----------------------------------------------------------------------------
			VTKM_CONT
			void SetMergeDuplicatePoints(bool on) { this->Worklet.SetMergeDuplicatePoints(on); }
			VTKM_CONT
			bool GetMergeDuplicatePoints() const { return this->Worklet.GetMergeDuplicatePoints(); }
		
			//-----------------------------------------------------------------------------
			VTKM_CONT
			void SetGenerateNormals(bool on) { this->GenerateNormals = on; }
			VTKM_CONT
			bool GetGenerateNormals() const { return this->GenerateNormals; }

			//-----------------------------------------------------------------------------
			VTKM_CONT
			void SetAddInterpolationEdgeIds(bool on) { this->AddInterpolationEdgeIds = on; }
			VTKM_CONT
			bool GetAddInterpolationEdgeIds() const { return this->AddInterpolationEdgeIds; }
			 //-----------------------------------------------------------------------------
			VTKM_CONT
			void SetComputeFastNormalsForStructured(bool on) { this->ComputeFastNormalsForStructured = on; }
			VTKM_CONT
			bool GetComputeFastNormalsForStructured() const { return this->ComputeFastNormalsForStructured; }

			//-----------------------------------------------------------------------------
			VTKM_CONT
			void SetComputeFastNormalsForUnstructured(bool on) { this->ComputeFastNormalsForUnstructured = on; }
			VTKM_CONT
			bool GetComputeFastNormalsForUnstructured() const {return this->ComputeFastNormalsForUnstructured;}

			//-----------------------------------------------------------------------------
			VTKM_CONT
			void SetNormalArrayName(const std::string& name) { this->NormalArrayName = name; }
			VTKM_CONT
			const std::string& GetNormalArrayName() const { return this->NormalArrayName; }
			 //-----------------------------------------------------------------------------
			
			private:
			  std::vector<vtkm::Float16> IsoValues;
			  bool GenerateNormals;
			  bool AddInterpolationEdgeIds;
			  bool ComputeFastNormalsForStructured;
			  bool ComputeFastNormalsForUnstructured;
			  std::string NormalArrayName;
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




int main() 
{
  vtkm::cont::DataSet inputDataSet, outputDataSet;
  vtkm::cont::DataSetBuilderUniform dataSetBuilder;
  vtkm::cont::DataSetFieldAdd dsf;

  vtkm::Id3 dims(3, 3, 3);
  vtkm::Id3 org(0,0,0);
  vtkm::Id3 spc(1,1,1);

  vtkm::Int64 N = 27;

  std::vector<vtkm::Float16> data(N);
  for (vtkm::Int64 i = 0; i < N; i++)
    data[i] = (float)i;
  vtkm::cont::ArrayHandle<vtkm::Float16> fieldData = vtkm::cont::make_ArrayHandle(data);

  std::string fieldName = "fieldData";
  inputDataSet = dataSetBuilder.Create(dims, org, spc);      
  dsf.AddPointField(inputDataSet, fieldName, fieldData);

 /* vtkm::filter::FilterFieldSquareFP16 fp16_filter;
  fp16_filter.SetActiveField(fieldName);  
  outputDataSet = fp16_filter.Execute(inputDataSet, vtkm::filter::PolicyFP16DataSet());
  */
  
   vtkm::filter::ContourFP16 contour;
   contour.SetGenerateNormals(false);
   //contour.SetMergeDuplicatePoints(false);
   contour.SetNumberOfIsoValues(1);
   contour.SetIsoValue(0, 0.5);
   contour.SetActiveField(fieldName);
   //vtkm::cont::DataSet ds_from_mc = contour.Execute(inputDataSet);

   return 0;

}
