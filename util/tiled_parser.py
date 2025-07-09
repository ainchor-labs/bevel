import pytmx

class TiledParser:
    def __init__(self, tmx_file_name):
        self.tmx_file_path = f"tilemaps/{tmx_file_name}"
        self.tmx_data = None

    def load(self):
        try:
            self.tmx_data = pytmx.TiledMap(self.tmx_file_path)
        except AttributeError as e:
            if "type" in str(e):
                print(f"TMX format compatibility issue: {e}")
                print("Try saving your map in an older TMX format version in Tiled editor")
                raise RuntimeError("TMX file format not compatible with PyTMX")
            else:
                raise
        except Exception as e:
            print(f"Error loading TMX file: {e}")
            raise

    def get_tiles(self):
        if not self.tmx_data:
            return []
        
        tiles = []
        try:
            for layer in self.tmx_data.visible_layers:
                if isinstance(layer, pytmx.TiledTileLayer):
                    for x, y, gid in layer:
                        tile = self.tmx_data.get_tile_image_by_gid(gid)
                        if tile:
                            px = x * self.tmx_data.tilewidth
                            py = y * self.tmx_data.tileheight
                            tiles.append({'image': tile, 'position': (px, py)})
        except Exception as e:
            print(f"Error processing tiles: {e}")
            return []
        
        return tiles