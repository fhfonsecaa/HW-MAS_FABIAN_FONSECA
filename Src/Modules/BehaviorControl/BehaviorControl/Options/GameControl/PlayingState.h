#define SINGLE_ROLE
#define getExtraTime() ( (theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber) ? 10000.f : 0.f)


option(PlayingState)
{
    initial_state(demo)
    {
        action
        {

#ifndef SINGLE_ROLE
            //  play normal game ||  free kick own team  ||  goal free kick own team  ||  kick in own team
            switch ((int)theGameInfo.setPlay)
            {
                case PLAY:
                    Stand();
                    break;
                case GOALFREEKICK:
                    Stand();
                    break;
                case FREEKICK:
                    Stand();
                    break;
                case CORNER:
                    Stand();
                    break;
                case SIDEKICK:
                    Stand();
                    break;    
                default:
                    exit (-1);
                    break;
            }

#else       
            // Stand();
            WalkToHomework();
            // KickToHomework();
#endif
        }
    }
}

