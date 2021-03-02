package database

import (
	"context"
	"github.com/gwaxG/robot_ws/backend/internal/common"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"
	"go.mongodb.org/mongo-driver/bson"
	"go.mongodb.org/mongo-driver/mongo"
	"go.mongodb.org/mongo-driver/mongo/options"
	"time"
)

type DataBase struct {
	client  		*mongo.Client
	ExpSeriesName	string
}

func (db *DataBase) Init (ExpSeriesName string) {
	db.ExpSeriesName = ExpSeriesName
	var err error
	db.client, err = mongo.NewClient(options.Client().ApplyURI("<ATLAS_URI_HERE>"))
	common.FailOnError(err)
}

func (db *DataBase) getContext () context.Context{
	ctx, _ := context.WithTimeout(context.Background(), 10*time.Second)
	return ctx
}

func (db *DataBase) addNewRolloutAnalytics(analytics structs.RolloutAnalytics) {
	ctx := db.getContext()
	err := db.client.Connect(ctx)
	common.FailOnError(err)
	defer db.client.Disconnect(ctx)
	experimentSeries := db.client.Database(db.ExpSeriesName)
	experimentCollection := experimentSeries.Collection(analytics.Experiment)
	_, err = experimentCollection.InsertOne(ctx, bson.D{
		{Key: "seq", Value: analytics.Seq},
		{Key: "sensors", Value: analytics.Sensors},
		{Key: "arm", Value: analytics.Arm},
		{Key: "angular", Value: analytics.Angular},
		{Key: "progress", Value: analytics.Progress},
		{Key: "reward", Value: analytics.Reward},
		{Key: "cog_deviation", Value: analytics.CogDeviation},
		{Key: "cog_height", Value: analytics.CogHeight},
		{Key: "accidents", Value: analytics.Accidents},
	})
	common.FailOnError(err)
}