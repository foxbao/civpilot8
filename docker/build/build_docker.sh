# copy the google third parties including protobuf, googletest into the docker
cp ../../third_party_civpilot.zip ./
cp -r ../../scripts ./

docker build  -f base.x86_64.dockerfile -t civ:civauto .

# rm -f third_party_civpilot.zip
# rm -rf scripts