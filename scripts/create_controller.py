import logging
import json
import sys
import subprocess
import os

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)

def generate_cpp_state_space(model_params):
    """
    Generates the C++ initialization code for a StateSpace plant 
    using the provided evaluated model parameters.
    """
    try:
        A = model_params['A']
        B = model_params['B']
        C = model_params['C']
        D = model_params['D']
        Ts = model_params.get('Ts', 0.01) # Default sample time

        # Basic validation
        if not all(k in model_params for k in ('A', 'B', 'C', 'D')):
            raise ValueError("Model parameters missing required matrices (A, B, C, D)")

        cpp_code = f"""// Auto-generated State-Space Model
#include "ControllerToolbox.h"

ctrl::StateSpace createIdentifiedPlant() {{
    const double Ts = {Ts};
    
    Eigen::MatrixXd A({len(A)}, {len(A[0])});
    A << {', '.join(map(str, sum(A, [])))};
    
    Eigen::MatrixXd B({len(B)}, {len(B[0])});
    B << {', '.join(map(str, sum(B, [])))};
    
    Eigen::MatrixXd C({len(C)}, {len(C[0])});
    C << {', '.join(map(str, sum(C, [])))};
    
    Eigen::MatrixXd D({len(D)}, {len(D[0])});
    D << {', '.join(map(str, sum(D, [])))};
    
    return ctrl::StateSpace(A, B, C, D, Ts);
}}
"""
        return cpp_code
    except Exception as e:
        logging.error(f"Failed to generate C++ matrices: {e}")
        raise

def create_controller(evaluated_model_path, output_cpp_path):
    """
    Reads an evaluated JSON model, generates the C++ code for the plant,
    and writes it to a file.
    """
    try:
        logging.info(f"Loading evaluated model from: {evaluated_model_path}")
        
        if not os.path.exists(evaluated_model_path):
            raise FileNotFoundError(f"Model file not found: {evaluated_model_path}")
            
        with open(evaluated_model_path, 'r') as f:
            model_params = json.load(f)
            
        logging.info("Validating model structure...")
        if 'fit_percentage' in model_params and model_params['fit_percentage'] < 60.0:
            logging.warning(f"Model fit is poor ({model_params['fit_percentage']}%). Controller performance may degrade.")

        logging.info("Generating C++ controller representation...")
        cpp_content = generate_cpp_state_space(model_params)
        
        logging.info(f"Writing C++ implementation to: {output_cpp_path}")
        with open(output_cpp_path, 'w') as f:
            f.write(cpp_content)
            
        return True
        
    except json.JSONDecodeError as e:
        logging.error(f"Invalid JSON in model file: {e}")
        raise
    except Exception as e:
        logging.error(f"Error creating controller: {e}")
        raise

if __name__ == "__main__":
    # Example usage
    # Expected JSON format: {"A": [[1.0, 0.1], [0.0, 0.9]], "B": [[0.01], [0.1]], "C": [[1.0, 0.0]], "D": [[0.0]], "Ts": 0.1, "fit_percentage": 92.5}
    input_model = "data/evaluated_model.json" 
    output_code = "src/IdentifiedPlant.cpp"
    
    # Create dummy data for example purposes if it doesn't exist
    if not os.path.exists("data"):
        os.makedirs("data")
    if not os.path.exists(input_model):
        dummy_model = {
            "A": [[0.95, 0.04], [-0.1, 0.9]], 
            "B": [[0.01], [0.5]], 
            "C": [[1.0, 0.0]], 
            "D": [[0.0]], 
            "Ts": 0.01,
            "fit_percentage": 85.2
        }
        with open(input_model, 'w') as f:
            json.dump(dummy_model, f, indent=4)
            
    if not os.path.exists("src"):
        os.makedirs("src")

    try:
        success = create_controller(input_model, output_code)
        if success:
            logging.info("Controller generation completed successfully.")
    except Exception as e:
        logging.critical(f"Failed to complete controller pipeline: {e}")
        sys.exit(1)
