# Docker w WSL bez Docker Desktop

Całość wymaga WSL2, a więc systemu Win 10 w wersji 2004 lub wyższej (jeżeli robicie regularne aktualizacje to raczej spełnione) lub Win 11.

## Instalacja WSL od zera

Osoby korzystające już z WSL, będą musiały zapewne dostosować pierwsze kroki do używanej przez siebie dystrybucji. Poradnik kierowany jest na wykorzystanie Ubuntu, ale zadziałało na co najmniej jednym Debianie. Wymaga Win 10 version 2004 lub wyżej lub Win 11.

Aby zainstalować WSL należy w wierszu polecenia (CMD) lub PowerShellu wpisać:

```ps
wsl --install -d Ubuntu-22.04
```

opcja `-d` definuje, którą z dostępnych dystrybucji zainstalować. Upewnij się, że zostanie wykorzystane WSL 2

```ps
wsl --set-version Ubuntu-22.04 2
```

Po instalacji powinien się uruchomić Linux proszący o podanie nazwy użytkownika i hasła do domyślnego użytkownika.

## Instalacja sterowników karty graficznej Nvidia

Jeśli posiadasz kartę graficzną Nvidia udaj się na [ich stronę](https://www.nvidia.com/download/index.aspx) i pobierz najnowszy sterownik dla swojego GPU.

## Instalacja Dockera

Zaktulizuj obecnie zainstalowane programy:

```bash
sudo apt update && sudo apt upgrade
```

Usuń pozostałości po Dockerze, jeśli bawiłeś się nim wcześniej:

```bash
sudo apt remove docker docker-engine docker.io containerd runc
```

Zainstaluj dependencje:

```bash
sudo apt install --no-install-recommends ca-certificates curl gnupg2
```

Pobierz i dodaj klucz GPG, a następnie dodaj repozytorium Dockera:

``` bash
. /etc/os-release
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/${ID}/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
```

``` bash
echo "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/${ID} ${VERSION_CODENAME} stable" \
| sudo tee /etc/apt/sources.list.d/docker.list
sudo apt update
```

Zainstaluj Dockera:

```bash
sudo apt install docker-ce docker-ce-cli containerd.io
```

Dodaj obecnego użytkownika do grupy `docker` aby móc uruchamiać kontenery bez konieczności używania sudo:

```bash
sudo usermod -aG docker $USER
```

W osobnej sesji uruchom daemona:

```bash
sudo dockerd
```

Natomiast w pierwszej sesji uruchom kontener testowy:

```bash
docker run --rm hello-world
```

**TODO** Uruchamianie deamone w tle.

## Dostęp do GPU

Dodaj klucz Nividi i repozytorium, następnie zainstaluj obsługę GPU:

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey \
| sudo gpg --dearmor -o /usr/share/keyrings/nvidia-docker-keyring.gpg
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-docker-keyring.gpg] https://#g' \
| sudo tee /etc/apt/sources.list.d/nvidia-docker.list
```

```bash
sudo apt update
sudo apt install -y nvidia-docker2
```

Możesz wrócić do głównego poradnika w celu zbudowania i uruchomienia dema.

**TODO** Nie zaobserwowałem działania GPU z poziomu menadżera zadań. Do sprawdzenia.

## Instalacja CUDA, wydaje się niepotrzebna

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-wsl-ubuntu.pin
sudo mv cuda-wsl-ubuntu.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.2.0/local_installers/cuda-repo-wsl-ubuntu-12-2-local_12.2.0-1_amd64.deb
sudo dpkg -i cuda-repo-wsl-ubuntu-12-2-local_12.2.0-1_amd64.deb
sudo cp /var/cuda-repo-wsl-ubuntu-12-2-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda
```

## Źródła

Dostęp 19.07.2023

<https://learn.microsoft.com/en-us/windows/wsl/install>
<https://learn.microsoft.com/en-us/windows/wsl/tutorials/gpu-compute>
<https://learn.microsoft.com/en-us/windows/wsl/tutorials/wsl-containers>

<https://dev.to/bowmanjd/install-docker-on-windows-wsl-without-docker-desktop-34m9>

<https://docs.docker.com/desktop/windows/wsl/>
<https://docs.docker.com/engine/install/ubuntu/>
<https://docs.docker.com/engine/install/linux-postinstall/>

<https://www.nvidia.com/download/index.aspx>
<https://docs.nvidia.com/cuda/wsl-user-guide/index.html#cuda-support-for-wsl-2>
<https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=WSL-Ubuntu&target_version=2.0&target_type=deb_local>
