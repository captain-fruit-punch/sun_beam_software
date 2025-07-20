import pandas as pd
import numpy as np

def parse_foil_data(file_path, verbose=False):
    # The actual CSV data starts at line 11 (index 10)
    # We need to skip the first 10 lines (metadata + empty line)
    foil_data = pd.read_csv(file_path, skiprows=10)

    if verbose:
        print("Successfully parsed the airfoil data!")
        print(f"Shape: {foil_data.shape}")
        print(f"Columns: {list(foil_data.columns)}")
        print("\nFirst few rows:")
        print(foil_data.head())

        print("\nData types:")
        print(foil_data.dtypes)

        print("\nSummary statistics:")
        print(foil_data.describe())
        # You can now use foil_data for your analysis
        # For example, to get the angle of attack and lift coefficient:
        print(f"\nAngle of attack range: {foil_data['Alpha'].min():.2f}° to {foil_data['Alpha'].max():.2f}°")
        print(f"Lift coefficient range: {foil_data['Cl'].min():.3f} to {foil_data['Cl'].max():.3f}")
        
    return foil_data

def interpolate_foil_data(foil_data, alpha, column_name):
    """
    Interpolates the specified column (e.g., 'Cl', 'Cd') in foil_data for the given alpha value(s).
    Returns a pandas DataFrame with columns ['Alpha', column_name].
    Raises ValueError if alpha is outside the available data range.
    """
    # Ensure the data is sorted by Alpha for interpolation
    foil_data_sorted = foil_data.sort_values('Alpha')
    # Remove duplicate Alpha values to avoid interpolation issues
    foil_data_sorted = foil_data_sorted.drop_duplicates(subset='Alpha')

    # Get the min and max of the available Alpha range
    alpha_min = foil_data_sorted['Alpha'].min()
    alpha_max = foil_data_sorted['Alpha'].max()

    # Convert alpha to a numpy array for consistent handling
    alpha_array = np.atleast_1d(alpha)

    # Check if any alpha values are outside the available range
    if np.any(alpha_array < alpha_min) or np.any(alpha_array > alpha_max):
        raise ValueError(
            f"Alpha value(s) {alpha_array} out of interpolation range: "
            f"{alpha_min:.2f}° to {alpha_max:.2f}°"
        )

    # Use numpy.interp for 1D linear interpolation
    interpolated_values = np.interp(
        alpha_array,
        foil_data_sorted['Alpha'],
        foil_data_sorted[column_name]
    )

    # Return as a DataFrame for easier downstream use
    interpolated_df = pd.DataFrame({
        'Alpha': alpha_array,
        column_name: interpolated_values
    })
    return interpolated_df

def get_c_l_from_foil_data(foil_data, alpha):
    interpolated_data = interpolate_foil_data(foil_data, alpha, 'Cl')
    return interpolated_data['Cl'].values[0]

def get_c_d_from_foil_data(foil_data, alpha):
    interpolated_data = interpolate_foil_data(foil_data, alpha, 'Cd')
    return interpolated_data['Cd'].values[0]

if __name__ == "__main__":
    foil_data = parse_foil_data("xf-ag03-il-50000.csv", verbose=True)
    try:
        interpolated_data = interpolate_foil_data(foil_data, alpha=2, column_name='Cl')
        print("Interpolated data:")
        print(interpolated_data)
    except ValueError as e:
        print(f"Interpolation error: {e}")
        
    c_l = get_c_l_from_foil_data(foil_data, 2)
    print(f"C_L at 2°: {c_l}")
    c_d = get_c_d_from_foil_data(foil_data, 2)
    print(f"C_D at 2°: {c_d}")
