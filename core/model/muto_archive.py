import os
import requests
import tarfile


class MutoArchive:
    def __init__(self) -> None:
        self.usr = os.path.expanduser("~")
        self.container = None

    def download_workspace(self, url):
        container_dir = f"{self.usr}/muto_workspaces"
        if not os.path.exists(container_dir):
            os.makedirs(container_dir)

        local_filename = url.split("/")[-1]
        print(f"local filename is: {local_filename}")
        local_filepath = os.path.join(container_dir, local_filename)
        print(f"local filepath is: {local_filepath}")

        with requests.get(url, stream=True) as r:
            r.raise_for_status()
            total_length = r.headers.get("content-length")
            print(f"total length is: {total_length}")

            with open(local_filepath, "wb") as f:
                if total_length is None:
                    f.write(r.content)
                else:
                    downloaded = 0
                    total_length = int(total_length)
                    for chunk in r.iter_content(chunk_size=8192):
                        if chunk:
                            f.write(chunk)
                            downloaded += len(chunk)
                            done = int(50 * downloaded / total_length)
                            print(
                                f"\r[{'=' * done}{' ' * (50-done)}] {downloaded / total_length:.2%}",
                                end="",
                            )

        print("\nWorkspace download complete.")
        return local_filepath

    def decompress_into_local(self, tar_path):
        try:
            if not tarfile.is_tarfile(tar_path):
                raise ValueError(f"{tar_path} is not a valid tar file.")

            muto_ws = f"{self.usr}/muto_workspaces"
            if not os.path.exists(muto_ws):
                os.mkdir(muto_ws)

            with tarfile.open(tar_path, "r") as tar:
                tar.extractall(path=muto_ws)
                print(f"Decompressed {tar_path} into {muto_ws}")
        except Exception as e:
            raise Exception(
                f"Exception while decompressing the workspace to the local machine: {e}"
            )
