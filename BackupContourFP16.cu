#include <vtkm/Types.h>
#include <vtkm/cont/DataSet.h>
#include <vtkm/cont/DataSetBuilderUniform.h>
#include <vtkm/cont/Invoker.h>
#include <vtkm/cont/ArrayHandle.h>
#include <vtkm/cont/Field.h>
#include <vtkm/cont/VariantArrayHandle.h>
#include <vtkm/cont/DataSetFieldAdd.h>
#include <vtkm/cont/CoordinateSystemFP16.h>
#include <vtkm/cont/ArrayHandleUniformPointCoordinatesFP16.h>
#include <vtkm/filter/FilterDataSetWithField.h>
#include <vtkm/filter/MapFieldPermutation.h>
//#include <vtkm/filter/Contour.h>
#include <vtkm/filter/PolicyDefault.h>
#include <vtkm/worklet/DispatcherMapField.h>
#include <vtkm/worklet/contour/FieldPropagation.h>
#include <vtkm/worklet/contour/CommonState.h>
#include <vtkm/worklet/contour/FlyingEdgesFP16.h>
#include <iostream>
#include <string>
namespace vtkm
{
	namespace worklet
	{
		namespace contour
		{
			struct DeduceCoordTypeFP16
			{
				template <typename... Args>
				  void operator()(
				    const vtkm::cont::ArrayHandle<vtkm::Vec3f_16, vtkm::cont::StorageTagUniformPointsFP16>& coords,
				    const vtkm::cont::CellSetStructured<3>& cells,
				    vtkm::cont::CellSetSingleType<>& result,
				    Args&&... args) const
				    {
				    	result = flying_edges::execute(cells, coords, std::forward<Args>(args)...);
				    }	
			};

			struct DeduceCellsTypeFP16
			{
			  template <typename CellSetType, typename CoordinateType, typename... Args>
			  void operator()(const CellSetType& cells, CoordinateType&& coordinateSystem, Args&&... args) const
			  {
			    vtkm::cont::CastAndCall(
			     coordinateSystem, contour::DeduceCoordTypeFP16{}, cells, std::forward<Args>(args)...);
			  }
			};
		}
		

		class ContourFP16
		{
			public:

			//----------------------------------------------------------------------------

			ContourFP16(bool mergeDuplicates = true)
				: SharedState(mergeDuplicates)
			{
			}
			
		       	//----------------------------------------------------------------------------
  			vtkm::cont::ArrayHandle<vtkm::Id2> GetInterpolationEdgeIds() const
  			{
    				return this->SharedState.InterpolationEdgeIds;
 		 	}

			//----------------------------------------------------------------------------
			void SetMergeDuplicatePoints(bool merge) { this->SharedState.MergeDuplicatePoints = merge; }

			//----------------------------------------------------------------------------
			bool GetMergeDuplicatePoints() const { return this->SharedState.MergeDuplicatePoints; }
		 
	       		//----------------------------------------------------------------------------
  			void ReleaseCellMapArrays() { this->SharedState.CellIdMap.ReleaseResources(); }		
			
			//----------------------------------------------------------------------------
  			vtkm::cont::ArrayHandle<vtkm::Id> GetCellIdMap() const { return this->SharedState.CellIdMap; }	
	
			//----------------------------------------------------------------------------
			  template <typename ValueType,
				    typename CellSetType,
				    typename CoordinateSystem,
				    typename StorageTagField,
				    typename CoordinateType,
				    typename StorageTagVertices>
			  vtkm::cont::CellSetSingleType<> Run(
			    const std::vector<ValueType>& isovalues,
			    const CellSetType& cells,
			    const CoordinateSystem& coordinateSystem,
			    const vtkm::cont::ArrayHandle<ValueType, StorageTagField>& input,
			    vtkm::cont::ArrayHandle<vtkm::Vec<CoordinateType, 3>, StorageTagVertices>& vertices)
			  {
			    this->SharedState.GenerateNormals = false;
			    vtkm::cont::ArrayHandle<vtkm::Vec<CoordinateType, 3>> normals;
			    /*vtkm::cont::CellSetSingleType<> outputCells = flying_edges::execute(
					    cells, 
					    coordinateSystem, 
					    isovalues, 
					    input, 
					    vertices, 
					    normals, 
					    this->SharedState);
			   */
			    vtkm::cont::CellSetSingleType<> outputCells;	
			    vtkm::cont::CastAndCall(cells, 
					   	contour::DeduceCellsTypeFP16{}, 
					   	coordinateSystem,
						outputCells,
						isovalues,
						input,
						vertices,
						normals,
						this->SharedState);
			    
			    return outputCells;
			  }

			  //----------------------------------------------------------------------------
			  template <typename ValueType,
				    typename CellSetType,
				    typename CoordinateSystem,
				    typename StorageTagField,
				    typename CoordinateType,
				    typename StorageTagVertices,
				    typename StorageTagNormals>
			  vtkm::cont::CellSetSingleType<> Run(
			    const std::vector<ValueType>& isovalues,
			    const CellSetType& cells,
			    const CoordinateSystem& coordinateSystem,
			    const vtkm::cont::ArrayHandle<ValueType, StorageTagField>& input,
			    vtkm::cont::ArrayHandle<vtkm::Vec<CoordinateType, 3>, StorageTagVertices>& vertices,
			    vtkm::cont::ArrayHandle<vtkm::Vec<CoordinateType, 3>, StorageTagNormals>& normals)
			  {
			    this->SharedState.GenerateNormals = true;
			    
			   vtkm::cont::CellSetSingleType<> outputCells;
			   vtkm::cont::CastAndCall(cells,
                                                contour::DeduceCellsTypeFP16{},        
                                                coordinateSystem,
                                                outputCells,
                                                isovalues,
                                                input,
                                                vertices,
                                                normals,
                                                this->SharedState);
			    /*
			    vtkm::cont::CellSetSingleType<> outputCells = flying_edges::execute(
					    cells,
                                            coordinateSystem,
                                            isovalues,
                                            input,
                                            vertices,
                                            normals,
                                            this->SharedState);
			    */
			    return outputCells;
			  }



			//----------------------------------------------------------------------------
  			template <typename ValueType, typename StorageType>
  			vtkm::cont::ArrayHandle<ValueType> ProcessPointField(
					const vtkm::cont::ArrayHandle<ValueType, StorageType>& input) const
			{
				using vtkm::worklet::contour::MapPointField;
				//vtkm::worklet::DispatcherMapField<MapPointField> applyFieldDispatcher;
				
				vtkm::cont::ArrayHandle<ValueType> output;
				/*applyFieldDispatcher.Invoke(this->SharedState.InterpolationEdgeIds,
						this->SharedState.InterpolationWeights,
						input,output);
    				*/
				return output;
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
			public:
			//-----------------------------------------------------------------------------
			using SupportedTypes = vtkm::List<vtkm::Float16>;

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
			template <typename T, typename StorageType, typename DerivedPolicy>
 	 		VTKM_CONT bool DoMapField(vtkm::cont::DataSet& result,
					const vtkm::cont::ArrayHandle<T, StorageType>& input,
					const vtkm::filter::FieldMetadata& fieldMeta,
					vtkm::filter::PolicyBase<DerivedPolicy>)
  			{
				VTKM_ASSERT(fieldMeta.IsPointField());
				vtkm::cont::ArrayHandle<T> fieldArray;
    				fieldArray = this->Worklet.ProcessPointField(input);
				result.AddField(fieldMeta.AsField(fieldArray));
				return true;
  			}

			//-----------------------------------------------------------------------------
		 	template <typename DerivedPolicy>
  			VTKM_CONT bool MapFieldOntoOutput(vtkm::cont::DataSet& result,
					const vtkm::cont::Field& field,
					vtkm::filter::PolicyBase<DerivedPolicy> policy)
			{
    				if (field.IsFieldPoint())
    				{
      					return this->FilterDataSetWithField<ContourFP16>::MapFieldOntoOutput(result, field, policy);
				}
    				else if (field.IsFieldCell())
    				{
      					vtkm::cont::ArrayHandle<vtkm::Id> permutation = this->Worklet.GetCellIdMap();
      					return vtkm::filter::MapFieldPermutation(field, permutation, result);
    				}
    		
				else if (field.IsFieldGlobal())
    				{
      					result.AddField(field);
      					return true;
			    	}
			    	else
			    	{
			      		return false;
			    	}
			  }	

			//-----------------------------------------------------------------------------
			template <typename T, typename StorageType, typename DerivedPolicy>
  vtkm::cont::DataSet DoExecute(const vtkm::cont::DataSet& input,
                                const vtkm::cont::ArrayHandle<T, StorageType>& field,
                                const vtkm::filter::FieldMetadata& fieldMeta,
                                vtkm::filter::PolicyBase<DerivedPolicy> policy)
			{

				if (fieldMeta.IsPointField() == false)
  				{
    					throw vtkm::cont::ErrorFilterExecution("Point field expected.");
  				}

  				if (this->IsoValues.size() == 0)
  				{
    					throw vtkm::cont::ErrorFilterExecution("No iso-values provided.");
  				}

  				// Check the fields of the dataset to see what kinds of fields are present so
  				// we can free the mapping arrays that won't be needed. A point field must
  				// exist for this algorithm, so just check cells.
  				const vtkm::Id numFields = input.GetNumberOfFields();
  				bool hasCellFields = false;
  				for (vtkm::Id fieldIdx = 0; fieldIdx < numFields && !hasCellFields; ++fieldIdx)
  				{
    					auto f = input.GetField(fieldIdx);
    					hasCellFields = f.IsFieldCell();
  				}


				 //get the cells and coordinates of the dataset
  				const vtkm::cont::DynamicCellSet& cells = input.GetCellSet();

  				const vtkm::cont::CoordinateSystemFP16& coords =
					input.GetCoordinateSystemFP16(this->GetActiveCoordinateSystemIndex());

				using Vec3HandleType = vtkm::cont::ArrayHandle<vtkm::Vec3f_16>;
  				Vec3HandleType vertices;
  				Vec3HandleType normals;
				
				vtkm::cont::DataSet output;
				vtkm::cont::CellSetSingleType<> outputCells;

				std::vector<T> ivalues(this->IsoValues.size());
				for (std::size_t i = 0; i < ivalues.size(); ++i)
				{
				    ivalues[i] = static_cast<T>(this->IsoValues[i]);
				}
				bool generateHighQualityNormals = IsCellSetStructured(cells)
    				? !this->ComputeFastNormalsForStructured
    				: !this->ComputeFastNormalsForUnstructured;

				if (this->GenerateNormals && generateHighQualityNormals)
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
					 outputCells = this->Worklet.Run(ivalues, 
							 	vtkm::filter::ApplyPolicyCellSet(cells, policy, *this),
                                                		coords.GetData(),
								field,
								vertices);

				}

				//Ruchi - Still have to add smoothening of faceted normals. Will work on it next	

				if (this->AddInterpolationEdgeIds)
  				{
    					vtkm::cont::Field interpolationEdgeIdsField(InterpolationEdgeIdsArrayName,
							vtkm::cont::Field::Association::POINTS,
							this->Worklet.GetInterpolationEdgeIds());
					output.AddField(interpolationEdgeIdsField);
				}
			 
				output.SetCellSet(outputCells);
				vtkm::cont::CoordinateSystemFP16 outputCoords("coordinates", vertices);
				output.AddCoordinateSystemFP16(outputCoords);

				if (!hasCellFields)
  				{
    					this->Worklet.ReleaseCellMapArrays();
  				}

				return output;	 
			}  

				
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

   vtkm::Int32 N = 27;

   std::vector<vtkm::Float16> data(N);
   for (vtkm::Int32 i = 0; i < N; i++)
     data[i] = (float)i;
   vtkm::cont::ArrayHandle<vtkm::Float16> fieldData = vtkm::cont::make_ArrayHandle(data);

   std::string fieldName = "fieldData";
   inputDataSet = dataSetBuilder.Create(dims, org, spc);      
   dsf.AddPointField(inputDataSet, fieldName, fieldData);

   vtkm::filter::ContourFP16 contour;
   contour.SetGenerateNormals(false);
   contour.SetMergeDuplicatePoints(false);
   contour.SetNumberOfIsoValues(1);
   vtkm::Float16 val = 0.5f;
   contour.SetIsoValue(0, val);
   contour.SetActiveField(fieldName);
   vtkm::cont::DataSet ds_from_mc = contour.Execute(inputDataSet,vtkm::filter::PolicyFP16DataSet());
    return 0;

}
