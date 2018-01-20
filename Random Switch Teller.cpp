std::string gameData;
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	if(gameData[0] == 'L')
	{
		//Put left auto code for the closest switch here. Brah, this is the one you want.....
	} else {
		//Put right auto code for the closest switch here. Brah, this is the one you want.....
	}
	if(gameData[1] == 'L')
	{
		//Put left auto code for the scale here, maybe might do the scale if we want
	} else {
		//Put right auto code for the scale here, maybe might do the scale if we want
	}
	if(gameData[2] == 'L')
	{
		//(Left) Just, no.
	} else {
		//(Right) We are not going for the other side at this point.
	}
;