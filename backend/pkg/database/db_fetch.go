package database

import (
	"context"
	"errors"
	"fmt"
	"go.mongodb.org/mongo-driver/bson"
)

type ResponseDbs struct {
	Data	[]string `json:"data"`
	Msg		string `json:"msg"`
}


type Coll struct{
	Name 	string `json:"name"`
	Fields 	[]string `json:"fields"`
}

type ResponseColls struct {
	Colls	[]Coll `json:"colls"`
	Msg		string `json:"msg"`
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

// We guaranty that one collection has the same set of fields because one rollout corresponds to one experiment.
func (db *DataBase) FetchColls(dst string) (*ResponseColls, error){
	// TODO Continue there
	db.changeDatabase(dst)
	collNames, err := db.database.ListCollectionNames(context.TODO(), bson.D{})
	if err != nil {
		return nil, errors.New("can not fetch collection names")
	}
	msg := ResponseColls{
		Colls: []Coll{},
		Msg:  "",
	}
	for _, collName := range collNames {
		fields := []string{}
		db.changeCollection(collName)
		//jsonStr, _ := db.collection.FindOne(context.TODO(), bson.M{}).Decode()
		jsonMap := make(map[string]interface{})
		// err = json.Unmarshal(jsonStr, &jsonMap)
		fmt.Println(err)
		fmt.Println(jsonMap)
		coll := Coll{Name: collName, Fields: fields}
		msg.Colls = append(msg.Colls, coll)
	}
	return &msg, nil
}