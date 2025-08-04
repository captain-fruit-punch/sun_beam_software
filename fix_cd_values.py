import pandas as pd

def fix_cd_values():
    """
    Fix the CD values in cd_vs_alpha.csv by applying the transformation: new_cd = 2 - old_cd
    This corrects for the Y-axis inversion where Y1=2 and Y2=0 were swapped.
    """
    
    # Read the original CSV file
    try:
        df = pd.read_csv('cd_vs_alpha.csv')
        print(f"Successfully read {len(df)} rows from cd_vs_alpha.csv")
    except FileNotFoundError:
        print("Error: cd_vs_alpha.csv not found in the current directory")
        return
    except Exception as e:
        print(f"Error reading file: {e}")
        return
    
    # Display original data statistics
    print(f"\nOriginal CD values:")
    print(f"Min: {df['cd'].min():.6f}")
    print(f"Max: {df['cd'].max():.6f}")
    print(f"Mean: {df['cd'].mean():.6f}")
    
    # Apply the transformation: new_cd = 2 - old_cd
    df['cd'] = 2 - df['cd']
    
    # Display corrected data statistics
    print(f"\nCorrected CD values:")
    print(f"Min: {df['cd'].min():.6f}")
    print(f"Max: {df['cd'].max():.6f}")
    print(f"Mean: {df['cd'].mean():.6f}")
    
    # Save the corrected data to a new file
    output_filename = 'cd_vs_alpha_corrected.csv'
    df.to_csv(output_filename, index=False)
    print(f"\nCorrected data saved to {output_filename}")
    
    # Show first few rows as verification
    print(f"\nFirst 5 rows of corrected data:")
    print(df.head())

if __name__ == "__main__":
    fix_cd_values() 