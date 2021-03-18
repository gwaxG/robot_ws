package database

import (
	"context"
	"errors"
	"github.com/gwaxG/robot_ws/backend/pkg/common"
	"go.mongodb.org/mongo-driver/bson"
	"strings"
)

type ResponseDbs struct {
	Data	[]string `json:"data"`
	Msg		string   `json:"msg"`
}

// Append new rollout analytics to corresponding database and collection
func (db *DataBase) FetchDbs() (*ResponseDbs, error){
	res, err := db.client.ListDatabaseNames(context.TODO(), bson.D{})
	if err != nil {
		return nil, errors.New("can not fetch database names")
	}
	out := []string{}
	for i:=0; i<len(res); i++ {
		if res[i] != "admin" && res[i] != "local" && res[i] != "config"{
			out = append(out, res[i])
		}
	}
	msg := &ResponseDbs{
		Data: out,
		Msg:  "",
	}
	return msg, nil
}

type ResponseColls struct {
	Colls	[]string `json:"colls"`
	Fields 	[]string `json:"fields"`
	Msg		string 	 `json:"msg"`
}

// We guaranty that one collection has the same set of fields because one rollout corresponds to one experiment.
// Same for databases. Convention: one database has collections where documents has same fields
func (db *DataBase) FetchColls(dst string) (*ResponseColls, error){
	db.changeDatabase(dst)
	collNames, err := db.database.ListCollectionNames(context.TODO(), bson.D{})
	if err != nil {
		return nil, errors.New("can not fetch collection names")
	}
	msg := &ResponseColls{
		Colls: []string{},
		Fields: []string{},
		Msg:  "",
	}
	// composing array of collection names
	for i, collName := range collNames {
		msg.Colls = append(msg.Colls, collName)
		// putting first collection first document field names into an array
		if i == 0 {
			db.changeCollection(collName)
			doc := map[string]interface{}{}
			common.FailOnError(db.collection.FindOne(context.TODO(), bson.M{}).Decode(&doc))
			for k, _ := range doc {
				if k != "_id" {
					msg.Fields = append(msg.Fields, k)
				}
			}
		}
	}
	return msg, nil
}

type ResponseVisualize struct {
	Data	map[string][]float64 	`json:"data"`
	Msg		string 	 				`json:"msg"`
}

/*
localhost:10000/visualize?database=test_exps&collection=test_rollout&fields=reward_progress_cogheight
*/
func (db *DataBase) FetchVisualize(dbName, collName, fieldString string) (*ResponseVisualize, error){
	defer func (){
		if r := recover(); r!=nil{

		}
	}()
	msg := ResponseVisualize{
		Data: map[string][]float64{},
		Msg: "",
	}
	db.changeDatabase(dbName)
	db.changeCollection(collName)
	var fields []string = strings.Split(fieldString, "_")

	for _, field := range fields {
		msg.Data[field] = []float64{}
	}

	cursor, err := db.collection.Find(context.TODO(), bson.M{})
	common.FailOnError(err)

	var docs []bson.M
	err = cursor.All(context.TODO(), &docs)
	common.FailOnError(err)

	for _, doc := range docs {
		for _, field := range fields {
			msg.Data[field] = append(msg.Data[field], doc[field].(float64))
		}
	}
	return &msg, nil
}



//
