import folium
import yaml


def visualize_bounding_boxes(seed_file="seed.yaml"):
    """
    Reads bounding box coordinates from a MapProxy seed configuration file and visualizes them on a Folium map.

    :author: Nelson Durrant
    :date: Mar 2025
    """

    try:
        with open(seed_file, "r") as f:
            config = yaml.safe_load(f)

        if "seed" not in config:
            print("No 'seed' section found in the configuration file.")
            return

        seed_data = config["seed"]

        if not seed_data:
            print("No seeds found in the configuration file.")
            return

        # Initialize the map with the center of the first bounding box (if available)
        first_bbox = next(iter(seed_data.values()), {}).get("bbox")
        if first_bbox:
            min_lon, min_lat, max_lon, max_lat = first_bbox
            m = folium.Map(
                location=[(min_lat + max_lat) / 2, (min_lon + max_lon) / 2],
                zoom_start=12,
            )
        else:
            m = folium.Map(location=[0, 0], zoom_start=2)  # Default map if no bbox

        # Iterate through the seeds and draw bounding boxes
        for seed_name, seed_config in seed_data.items():
            if "bbox" in seed_config:
                min_lon, min_lat, max_lon, max_lat = seed_config["bbox"]
                folium.Rectangle(
                    bounds=[(min_lat, min_lon), (max_lat, max_lon)],
                    color="red",
                    fill=False,
                    popup=seed_name,  # Add popup with seed name
                ).add_to(m)
            else:
                print(f"Seed '{seed_name}' has no bbox defined.")

        # Save the map to an HTML file
        m.save("bounding_boxes_map.html")
        print("Bounding boxes visualization saved to bounding_boxes_map.html")

    except FileNotFoundError:
        print(f"Error: Configuration file '{seed_file}' not found.")
    except yaml.YAMLError as e:
        print(f"Error: Failed to parse YAML configuration: {e}")
    except Exception as e:
        print(f"An unexpected error occured: {e}")


if __name__ == "__main__":
    visualize_bounding_boxes()
